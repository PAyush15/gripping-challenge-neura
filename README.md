# gripping-challenge-neura

Franka Panda Hand/Gripper MVP Skill
Technical Challenge (Option 1)
Author: Ayush Vasantbhai Patel

Contact: patelayushvasantbhai@gmail.com

1. Summary
This repository contains a minimal manipulation skill implemented for the Franka Panda arm using a two-finger parallel gripper in PyBullet.

The logic is driven by a Finite State Machine (FSM) that utilizes motion primitives (IK-based approach, descend, lift). A key feature of this implementation is its Failure Detection and Mitigation system, which identifies unsuccessful grasps and triggers a recovery routine to adapt parameters and retry the task.

2. Tech Stack & Environment
Simulator: PyBullet

OS: Ubuntu 22.04

Python: 3.10

Robot Model: Franka Panda + Parallel Gripper (URDF)

Assets: pybullet_data (plane, table, small cube)

Control: Inverse Kinematics (IK) + Joint Position Control

3. Sensing & Interaction Logic
Sensing Assumptions: The system currently pulls object poses (ground truth) directly from the simulator. In production, this would be replaced by a perception module (e.g., RGB-D) subject to noise.

Policy: The behavior is deterministic, managed via a structured FSM.

4. Code Structure
Plaintext
.
├── src/
│   ├── env.py         # Simulation setup (world, table, cube, Panda loading)
│   ├── kinematics.py  # Grasp target generation & End-Effector (EE) helpers
│   ├── motion.py      # Motion primitives (IK move, open/close gripper)
│   ├── policy.py      # FSM policy (states, transitions, recovery logic)
│   └── main.py        # CLI entry point and demo orchestration
├── requirements.txt   # Project dependencies
└── README.md          # Project documentation
5. Interaction Logic (FSM)
The robot transitions through the following states to complete a grasp:

RESET: Initialize robot and environment.

APPROACH: Move EE to a safe pre-grasp pose above the object.

DESCEND: Lower EE to the computed grasp pose.

CLOSE: Actuate gripper fingers.

VERIFY_GRASP: Check for contact points between the robot and object.

LIFT/HOLD: Elevate the object and maintain position.

SUCCESS: Mission completed.

RECOVER: Triggered if VERIFY_GRASP fails.

6. Failure Mode and Mitigation
Failure Detected: Unsuccessful grasp (insufficient contact points) caused by pose errors or improper grasp depth.

Recovery Strategy:
Open & Reset: Gripper opens and moves to a safe clearance height.

Rescan: Re-calculates object pose (simulating a sensor refresh).

Parameter Adaptation: Grasp_Z is incrementally reduced (deeper descend) per retry, bounded to prevent table collision.

Retry: Re-enters the APPROACH state (limited to 3 retries).

7. AI Tools Usage
AI tools were utilized for:

Clarifying PyBullet API syntax.

Structuring modular Python file organization.

Implementing argparse for the CLI.

Assisting with documentation formatting.

Note: All core control logic, FSM transitions, and recovery strategies were manually implemented.

8. Installation & Usage
Setup Environment
Bash
# Clone the repository
git clone https://github.com/PAyush15/gripping-challenge-neura.git
cd gripping-challenge-neura

# Create and activate virtual environment
python3 -m venv pybullet_sim_env
source pybullet_sim_env/bin/activate

# Install dependencies
pip install -r requirements.txt
Run Demos
Standard Success Demo:

Bash
python -m src.main
Recovery/Failure Mitigation Demo:

Bash
python -m src.main --recovery-demo
9. Future Roadmap
Perception Integration: Replace ground-truth poses with noisy sensor data.

Slip Detection: Monitor contact forces during the HOLD state.

Robot Agnostic: The policy is designed to be robot-agnostic, requiring only an EE controller and gripper primitives.
