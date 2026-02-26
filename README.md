# NAO Upper-Body Pose Imitation with MediaPipe

This project implements real-time upper-body pose imitation for the NAO humanoid robot using a standard webcam and MediaPipe Pose.
Human upper-body motion is captured on  Windows , transmitted via  UDP  to  WSL (Linux) , and safely executed on the  NAO robot  using NAOqi.

The system focuses on:
- Head motion imitation
- Shoulder lifting & horizontal opening (T-pose)
- Elbow flexion ( ElbowRoll ) and forearm swing ( ElbowYaw )
- Safe balance-aware execution on NAO

---

## 1. System Architecture

```java
Human → RGB Camera → Pose Client (Windows)
                            |
                            |  UDP (JSON)
                            ↓
                   Pose Server (WSL / Linux)
                            |
                            ↓
                       NAO Robot
```


-  Windows : Webcam + MediaPipe Pose → joint angle estimation
-  UDP : Lightweight real-time communication
-  WSL : NAO motion server & safety controller
-  NAO : Executes upper-body motion with balance constraints

---

## 2. Runtime Environment

### 2.1 Hardware

- NAO robot (NAO V5 / V6 recommended)
- PC with webcam
- Stable local network (PC ↔ NAO)

---

### 2.2 Operating Systems

| Component | OS |
|---------|----|
| Pose Client | Windows 10 / 11 |
| Pose Server | WSL2 (Ubuntu 20.04 / 22.04) |
| Robot | NAOqi OS |

---

### 2.3 Software Versions

#### Windows
- Python  3.8 – 3.10 
- OpenCV
- MediaPipe

#### WSL (Linux)
- Python  2.7 or 3.x  (NAOqi compatible)
- NAOqi Python SDK
- qi framework

---

## 3. Dependencies

### 3.1 Windows (Pose Client)

Install dependencies with:

```bash
pip install opencv-python mediapipe numpy
```

### 3.2 WSL (Pose Server)

Make sure NAOqi SDK is correctly installed and accessible.

Required modules:
- qi
- ALMotion
- ALRobotPosture
- ALTextToSpeech

The NAOqi Python environment must match the robot firmware version.

## 4. Project Structure
```base
nao_pose_mimic/
├── pose_client_full.py              # Windows: MediaPipe → UDP client
├── nao_server_balance_safe_full.py  # WSL: UDP → NAO motion server
├── README.md
```

## 5. Configuration

### 5.1 Windows Client
Edit in pose_client.py:
```python
WSL_IP = "172.27.125.13"   # IP of WSL (check with `ip addr` in WSL)
WSL_PORT = 5005
CAMERA_INDEX = 0          # Webcam index
```
### 5.2 Windows Client
Edit in nao_server.py
```python
ROBOT_IP = "192.168.2.121"   # NAO robot IP
ROBOT_PORT = 9559
```

## 6. Running the System
### Step 1 — Start NAO Server (WSL)
```base
cd ~/nao_pose_mimic
python3 nao_server_balance_safe_full.py
```
The robot will:
- Wake up
- Move to StandInit
- Enable safe balance mode

### Step 2 — Start Pose Client (Windows)
```base
python pose_client_full.py
```
- Webcam window opens
- Human upper-body pose is tracked
- Robot imitates head, shoulders, and elbows in real time
Press ESC to exit.

## 7. Control Mapping Overview

The following table summarizes how human upper-body motions are mapped to NAO joints.

| Human Motion | NAO Joint |
|-------------|----------|
| Head left / right | `HeadYaw` |
| Head up / down | `HeadPitch` |
| Arm raise (forward / upward) | `ShoulderPitch` |
| Arm horizontal opening (T-pose) | `ShoulderRoll` |
| Elbow bending | `ElbowRoll` |
| Forearm swing (front / back) | `ElbowYaw` |

The combination of  ShoulderPitch + ShoulderRoll  enables both vertical lifting and horizontal arm opening, while  ElbowRoll + ElbowYaw  together approximate human forearm motion.

---

## 8. Safety Design

To ensure stable and safe robot behavior, several safety mechanisms are implemented:

-  Joint angle clamping   
  All joint commands are constrained within NAO’s hardware-safe ranges.

-  Separated speed control   
  Different speed factors are used for head, arms, and legs to avoid abrupt motion.

-  Balance-aware posture control   
  The robot remains in `StandInit` posture with whole-body balance enabled.

-  Limited torso influence   
  Torso inclination is approximated via hip joints with small amplitude only.

-  Motion smoothing   
  Exponential moving average (EMA) filtering is applied to reduce jitter while preserving responsiveness.

-  Stiffness enforcement   
  Arm and head stiffness are explicitly enabled to prevent joints from becoming unresponsive.

These design choices significantly reduce the risk of instability or falling during imitation.

---

## 9. Known Limitations

Despite achieving real-time upper-body imitation, several limitations remain:

- The NAO shoulder structure limits  true vertical arm raising  above the head.
- Human forearm motion is approximated using `ElbowYaw`, which does not fully match human anatomy.
- No inverse kinematics (IK) solver is used; joint mapping is based on geometric heuristics.
- The system performs  upper-body-only imitation ; legs are controlled solely for balance.
- Fast or highly dynamic human motions may be attenuated due to safety clamping and smoothing.

These limitations are inherent to both the robot hardware and the simplified control strategy.

---

## 10. Future Improvements

Several extensions can further improve the system:

- Integration of  inverse kinematics (IK)  with balance constraints.
- Adaptive motion scaling based on detected human movement speed.
- Gesture-level recognition (e.g., waving, pointing, raising hand).
- Learning-based pose mapping using neural networks.
- Full-body imitation including leg motion.
- Online calibration to adapt to different human body proportions.

These improvements would enhance realism, robustness, and interaction quality.

---
