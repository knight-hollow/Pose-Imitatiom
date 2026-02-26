import cv2
import mediapipe as mp
import numpy as np
import socket
import json
import math

# -----------------------------------
# CONFIGURATION
# -----------------------------------
WSL_IP = "172.27.125.13" 
WSL_PORT = 5005
CAMERA_INDEX = 0

# -----------------------------------
mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------- helper functions ----------
def vec(a, b):
    return np.array([b.x - a.x, b.y - a.y, b.z - a.z], dtype=np.float32)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def unit(v):
    n = float(np.linalg.norm(v)) + 1e-6
    return v / n

class Smoother:
    def __init__(self, alpha=0.8):
        self.alpha = alpha
        self.prev = {}

    def smooth(self, d):
        out = {}
        for k, v in d.items():
            if k not in self.prev:
                out[k] = v
            else:
                out[k] = self.alpha * self.prev[k] + (1 - self.alpha) * v
        self.prev = out
        return out

smoother = Smoother(alpha=0.80)

# ----------------------------------------------------
# TORSO INCLINATION COMPUTATION (ROLL & PITCH)
# ----------------------------------------------------
def compute_torso(lm):
    LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
    RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    LH = lm[mp_pose.PoseLandmark.LEFT_HIP.value]
    RH = lm[mp_pose.PoseLandmark.RIGHT_HIP.value]

    shoulder_mid = np.array([(LS.x + RS.x) / 2,
                             (LS.y + RS.y) / 2,
                             (LS.z + RS.z) / 2], dtype=np.float32)
    hip_mid = np.array([(LH.x + RH.x) / 2,
                        (LH.y + RH.y) / 2,
                        (LH.z + RH.z) / 2], dtype=np.float32)

    v = shoulder_mid - hip_mid

    roll  = clamp(float(v[0]) * 1.2, -0.25, 0.25)
    pitch = clamp(float(-v[1]) * 1.2, -0.25, 0.25)
    return roll, pitch

# ----------------------------------------------------
# UPPER-BODY JOINT MAPPING
# ----------------------------------------------------
def compute_upper_body_angles(lm):
    angles = {}

    LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
    RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    LE = lm[mp_pose.PoseLandmark.LEFT_ELBOW.value]
    RE = lm[mp_pose.PoseLandmark.RIGHT_ELBOW.value]
    LW = lm[mp_pose.PoseLandmark.LEFT_WRIST.value]
    RW = lm[mp_pose.PoseLandmark.RIGHT_WRIST.value]
    NOSE = lm[mp_pose.PoseLandmark.NOSE.value]

    # ---------- Head ----------
    shoulder_mid = np.array([(LS.x + RS.x) / 2.0,
                             (LS.y + RS.y) / 2.0,
                             (LS.z + RS.z) / 2.0], dtype=np.float32)
    nose_vec = np.array([NOSE.x - shoulder_mid[0],
                         NOSE.y - shoulder_mid[1],
                         NOSE.z - shoulder_mid[2]], dtype=np.float32)

    HEAD_YAW_GAIN = 3.5
    HEAD_PITCH_GAIN = 3.5

    angles["HeadYaw"] = clamp(-float(nose_vec[0]) * HEAD_YAW_GAIN, -1.8, 1.8)
    angles["HeadPitch"] = clamp(float(nose_vec[1]) * HEAD_PITCH_GAIN, -0.9, 0.9)

    # ---------- Upper-arm vectors ----------
    vL = vec(LS, LE)   # shoulder -> elbow
    vR = vec(RS, RE)
    uL = unit(vL)
    uR = unit(vR)

    # ---------- ShoulderPitch (robust) ----------
    # MediaPipe image coords: y increases downward, so "down" is +Y.
    down = np.array([0.0, 1.0, 0.0], dtype=np.float32)

    def shoulder_pitch_from_u(u):
        dot = float(np.clip(np.dot(u, down), -1.0, 1.0))
        ang = math.acos(dot)  # 0..pi
        BASE = 1.55
        GAIN = 1.45
        return BASE - GAIN * ang

    L_pitch = shoulder_pitch_from_u(uL)
    R_pitch = shoulder_pitch_from_u(uR)

    angles["LShoulderPitch"] = clamp(L_pitch, -2.0, 2.0)
    angles["RShoulderPitch"] = clamp(R_pitch, -2.0, 2.0)

    # ---------- Intention detection: side raise vs forward raise ----------
    # Key fix: Only treat as side raise if |x| dominates |z|.
    def is_side_raise(u):
        # horizontal-ish AND more sideways than forward
        return (abs(float(u[1])) < 0.30) and (abs(float(u[0])) > abs(float(u[2])))

    def is_forward_raise(u):
        # more forward than sideways
        return abs(float(u[2])) > abs(float(u[0]))

    # ---------- ShoulderRoll ----------
    # Left outward  -> +roll ; Right outward -> -roll
    ROLL_GAIN = 6.0

    L_roll = ROLL_GAIN * float(uL[0])
    R_roll = -ROLL_GAIN * float(uR[0])   # sign fix for right arm

    # If arm is mainly forward, suppress roll strongly (prevents accidental T-pose)
    if is_forward_raise(uL):
        L_roll *= 0.15
    if is_forward_raise(uR):
        R_roll *= 0.15

    # If true side raise detected, push towards near-max opening
    if is_side_raise(uL):
        L_roll = max(L_roll, 1.25)
        # also bias pitch for stable T-pose opening
        angles["LShoulderPitch"] = clamp(angles["LShoulderPitch"], 0.6, 1.15)

    if is_side_raise(uR):
        R_roll = min(R_roll, -1.25)
        angles["RShoulderPitch"] = clamp(angles["RShoulderPitch"], 0.6, 1.15)

    angles["LShoulderRoll"] = clamp(L_roll, 0.0, 1.56)
    angles["RShoulderRoll"] = clamp(R_roll, -1.56, 0.0)

    # ---------- ElbowRoll ----------
    def elbow_roll(a, b, c, left=True):
        v1 = vec(b, a)  # elbow -> shoulder
        v2 = vec(b, c)  # elbow -> wrist
        v1n = unit(v1)
        v2n = unit(v2)

        dot = float(np.clip(np.dot(v1n, v2n), -1.0, 1.0))
        theta = math.acos(dot)
        phi = math.pi - theta  # 0 straight, larger bent

        phi0 = math.radians(8.0)
        phimax = math.radians(120.0)
        phi = max(0.0, min(phi, phimax))

        if phi <= phi0:
            t = 0.0
        else:
            t = (phi - phi0) / (phimax - phi0)
            t = max(0.0, min(t, 1.0))

        if left:
            min_ang = -1.5
            max_ang = -0.03
            ang = max_ang - t * (max_ang - min_ang)
        else:
            min_ang = 0.03
            max_ang = 1.5
            ang = min_ang + t * (max_ang - min_ang)

        return ang

    angles["LElbowRoll"] = elbow_roll(LS, LE, LW, left=True)
    angles["RElbowRoll"] = elbow_roll(RS, RE, RW, left=False)

    angles["LElbowYaw"] = 0.0
    angles["RElbowYaw"] = 0.0

    return angles

# ----------------------------------------------------
# SEND DATA
# ----------------------------------------------------
def send_data(angles, torso_roll, torso_pitch):
    angles_clean = {k: float(v) for k, v in angles.items()}
    msg = {
        "angles": angles_clean,
        "torso": {"roll": float(torso_roll), "pitch": float(torso_pitch)}
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (WSL_IP, WSL_PORT))

# ----------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    with mp_pose.Pose(min_detection_confidence=0.6,
                      min_tracking_confidence=0.6) as pose:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = pose.process(rgb)

            if res.pose_landmarks:
                mp_draw.draw_landmarks(frame, res.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                lm = res.pose_landmarks.landmark

                torso_roll, torso_pitch = compute_torso(lm)
                angles = compute_upper_body_angles(lm)
                angles = smoother.smooth(angles)

                send_data(angles, torso_roll, torso_pitch)

            cv2.imshow("Upper-body imitation", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
