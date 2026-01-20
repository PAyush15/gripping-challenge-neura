import pybullet as p
from dataclasses import dataclass

from .kinematics import compute_grasp_poses, down_orientation, get_object_pose
from .motions import move_ee_pose, open_gripper, close_gripper, step_sim

@dataclass
class PolicyConfig:
    max_retries: int = 3
    grasp_z_offset: float = 0.03
    grasp_z_min: float = 0.01
    grasp_z_step: float = 0.005
    pregrasp_steps: int = 240
    descend_steps: int = 240
    close_steps: int = 240
    lift_steps: int = 480
    hold_steps: int = 480
    min_contacts: int = 1


def verify_grasp(robot_id, box_id, min_contacts=1):

    contact_points = p.getContactPoints(bodyA=robot_id, bodyB=box_id)
    return len(contact_points) >= min_contacts, len(contact_points)

def run_fsm(robot_id, box_id, arm_joints, finger_joints, ee_link, cfg: PolicyConfig, recovery_demo: bool = False):
    state = "RESET"
    retries = 0
    grasp_z = cfg.grasp_z_offset
    orientation = down_orientation()
    first_attempt = True

    while state not in ("SUCCESS", "FAIL"):
        # RESET
        if state == "RESET":
            open_gripper(robot_id, finger_joints, open_width=0.04, force=50)
            step_sim(240)
            state = "APPROACH"

        # APPROACH
        elif state == "APPROACH":
            pregrasp, grasp, lift = compute_grasp_poses(box_id, grasp_z_offset=grasp_z)
            move_ee_pose(robot_id, ee_link, arm_joints, pregrasp, orientation, steps=cfg.pregrasp_steps)
            state = "DESCEND"

        # DESCEND
        elif state == "DESCEND":
            (ox, oy, oz), _ = get_object_pose(box_id)
            grasp_pos = [ox, oy, oz + grasp_z]
            if recovery_demo and first_attempt:
                grasp_pos[0] += 0.06
                grasp_pos[1] += 0.02
            move_ee_pose(robot_id, ee_link, arm_joints, grasp_pos, orientation, steps=cfg.pregrasp_steps)
            state = "CLOSE"

        # CLOSE
        elif state == "CLOSE":
            close_gripper(robot_id, finger_joints, close_width=0.0, force=100)
            step_sim(cfg.close_steps)
            state = "VERIFY_GRASP"

        # VERIFY_GRASP
        elif state == "VERIFY_GRASP":
            ok, n_contacts = verify_grasp(robot_id, box_id, min_contacts=cfg.min_contacts)
            print(f"[VERIFY_GRASP] contacts = {n_contacts}, ok={ok}")
            first_attempt = False
            state = "LIFT" if ok else "RECOVER"

        # LIFT
        elif state == "LIFT":
            (ox, oy, oz), _ = get_object_pose(box_id)
            lift_pos = [ox, oy, oz + 0.3]
            move_ee_pose(robot_id, ee_link, arm_joints, lift_pos, orientation, steps=cfg.lift_steps)
            state = "HOLD"

        # SUCCESS
        elif state == "HOLD":
            step_sim(cfg.hold_steps)
            state = "SUCCESS"

        # RECOVER
        elif state == "RECOVER":
            retries += 1
            if retries > cfg.max_retries:
                state = "FAIL"
                continue

            print(f"[RECOVER] retry={retries}. Opening, backing off and adjusting grasp_z")

            open_gripper(robot_id, finger_joints, open_width=0.04, force=50)
            step_sim(120)

            (ox, oy, oz), _ = get_object_pose(box_id)
            safe = [ox, oy, oz + 0.25]
            move_ee_pose(robot_id, ee_link, arm_joints, safe, orientation, steps=240)

            grasp_z = max(cfg.grasp_z_min, grasp_z - cfg.grasp_z_step)
            print(f"[RECOVER] new grasp_z_offset={grasp_z:.3f}")

            state = "APPROACH"
        
        else: 
            state = "FAIL"

    return state
