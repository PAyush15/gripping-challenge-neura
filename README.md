# gripping-challenge-neura

# Franka Panda Hand/Gripper MVP Skill
### Technical Challenge – Hand/Gripper MVP Skill (Option 1)

**Author:** Ayush Vasantbhai Patel  
**Contact:** [patelayushvasantbhai@gmail.com](mailto:patelayushvasantbhai@gmail.com)

---



https://github.com/user-attachments/assets/b929ab28-734e-4327-954a-66e02b36ad26




## 1. Summary
I implemented a minimal manipulation skill that grasps a single object (a cube on a table) using the Franka Panda two-finger parallel gripper in PyBullet. Finite state machine (FSM) is used as the main logic with clear motion primitives (IK-based approach/descend/lift and open/close). One failure mode (bad grasp due to pose error) is detected and mitigated via a recovery routine that retries with adapted parameters.

---

## 2. Tools, Technologies and Gripper Model

| Category | Specification |
| :--- | :--- |
| **Simulator** | PyBullet (OS: Ubuntu 22.04, Python 3.10) |
| **Robot Model** | Franka Panda arm + two-finger gripper (URDF from pybullet_data) |
| **Scene** | Plane + table + cube_small (pybullet_data assets) |
| **Control** | PyBullet inverse kinematics + joint position control |

---

## 3. Sensing Assumptions and Interaction Logic
* **Sensing Assumption:** The object pose is taken from the simulator (ground truth). In a real system, this would be provided by a perception module (e.g., RGB-D pose estimate) and would include noise.
* **Policy:** The behavior is implemented via FSM and is deterministic.

---

## 4. Code Structure

```text
src/
├── env.py         # Simulation setup, world (table/cube), Panda loading, joint/link indexing
├── kinematics.py  # Grasp target generation (pregrasp/grasp/lift) and EE orientation helpers
├── motion.py      # Motion primitives (step_sim, IK move_ee_pose, open/close gripper)
├── policy.py      # FSM policy (states, transitions, grasp verification, recovery adaptation)
└── main.py        # CLI entry point, wiring, demo mode selection
```

## 5. How to Run

### 1. Installation and set-up
```bash
# Clone the repository
git clone https://github.com/PAyush15/gripping-challenge-neura.git
cd gripping-challenge-neura

# Setup virtual environment
python3 -m venv pybullet_sim_env
source pybullet_sim_env/bin/activate
pip install -r requirements.txt
```

### Running the demos
```
# Run (success demo):
python -m src.main

# Run (detected failure and recovery demo):
python -m src.main --recovery-demo
```

## 6. Interaction Logic (FSM)

The manipulation skill is governed by a Finite State Machine (FSM) to ensure structured transitions between motion primitives.

| State | Description |
| :--- | :--- |
| **APPROACH** | Moves the end-effector to a safe pre-grasp position above the target object. |
| **DESCEND** | Lowers the gripper to the specific grasp pose (Object Z + offset). |
| **CLOSE** | Actuates the parallel fingers to secure the object. |
| **VERIFY_GRASP** | Checks for valid contact points to detect if the grasp was successful. |
| **LIFT / HOLD** | Elevates the object to a target height and maintains position. |
| **RECOVER** | Triggered upon grasp failure to adjust parameters and retry. |



---

## 7. Failure Mode and Mitigation (Recovery Demo)

**Failure Condition:** The system detects an unsuccessful grasp if contact sensors report insufficient pressure/points after the `CLOSE` state.

### Recovery Workflow:
1. **Retract:** Open gripper and return to a safe clearance height.
2. **Re-evaluate:** Re-sample object pose from the environment.
3. **Adapt:** Incrementally reduce the `grasp_z_offset` (descend deeper) to ensure better finger-to-object contact.
4. **Retry:** Re-initiate the sequence from the **APPROACH** state (Max 3 retries).

---

## 8. Use of AI Tools During Development

AI tools were utilized for the following tasks:
* Clarifying specific **PyBullet API** function signatures.
* Suggesting a **modular directory structure** (`src/` organization).
* Assisting with the implementation of **CLI arguments** (`argparse`) for demo selection.
* *Note: All logic regarding the state machine, failure detection, and recovery adaptation was manually developed.*
