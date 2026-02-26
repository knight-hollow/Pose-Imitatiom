import qi
import socket
import json
import sys
import time

ROBOT_IP = "192.168.2.121"
ROBOT_PORT = 9559

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

def main():

    try:
        app = qi.Application(["BalanceServer",
                             "--qi-url=tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT)])
    except Exception as e:
        print("Connection error:", e)
        sys.exit(1)

    app.start()
    session = app.session

    motion = session.service("ALMotion")
    posture = session.service("ALRobotPosture")
    tts = session.service("ALTextToSpeech")

    tts.say("Starting pose imitation.")

    motion.wakeUp()
    posture.goToPosture("StandInit", 0.8)

    # --------------------------------------------
    # ENABLE WHOLE BODY BALANCER
    # --------------------------------------------
    motion.wbEnable(True)
    motion.wbGoToBalance("Legs", 2.0)

    # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print("[SAFE BALANCE SERVER] Listening...")

    try:
        while True:
            data, addr = sock.recvfrom(8192)

            try:
                msg = json.loads(data.decode("utf-8"))
            except:
                continue

            # --------------------------------
            # Upper-body angles (arms + head)
            # --------------------------------
            angles = msg["angles"]
            joint_names = list(angles.keys())
            joint_vals = [float(angles[n]) for n in joint_names]

            try:
                motion.setAngles(joint_names, joint_vals, 0.35)
            except Exception as e:
                print("setAngles error:", e)

            # --------------------------------
            # Torso “inclination” via hip joints
            # --------------------------------
            torso_roll = float(msg["torso"]["roll"])
            torso_pitch = float(msg["torso"]["pitch"])

            # SAFETY FIRST: clamp values
            torso_roll = max(-0.15, min(0.15, torso_roll))
            torso_pitch = max(-0.15, min(0.15, torso_pitch))

            # Map torso roll → hip roll
            LHipRoll = torso_roll * 0.5
            RHipRoll = -torso_roll * 0.5

            # Map torso pitch → hip pitch
            LHipPitch = torso_pitch * 0.6
            RHipPitch = torso_pitch * 0.6

            leg_names = ["LHipRoll", "RHipRoll", "LHipPitch", "RHipPitch"]
            leg_vals = [LHipRoll, RHipRoll, LHipPitch, RHipPitch]

            try:
                motion.setAngles(leg_names, leg_vals, 0.15)
            except Exception as e:
                print("hip control error:", e)

    except KeyboardInterrupt:
        tts.say("Stopping imitation.")
    finally:
        motion.wbEnable(False)
        motion.rest()
        sock.close()


if __name__ == "__main__":
    main()
