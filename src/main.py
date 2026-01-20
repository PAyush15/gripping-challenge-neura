from src.env import setup_sim, load_world, load_panda, get_panda_indices, reset_home
from src.policy import run_fsm, PolicyConfig
from src.motions import step_sim
import argparse
import time


def main():
    parse_argument = argparse.ArgumentParser(description="Panda grasp argument to demonstrate sucess or recovery/failure behaviour")
    parse_argument.add_argument(
        "--recovery-demo",
        action="store_true"
    )
    args = parse_argument.parse_args()

    setup_sim(gui=True)

    plane_id, table_id, box_id = load_world(table_pos=(0.9, 0.0, 0.0), box_pos=(0.3, 0.0, 0.65))
    robot_id = load_panda(base_pos=(0.0, 0.0, 0.0), fixed_base=True)

    time.sleep(3)

    arm_joints, finger_joints, ee_link = get_panda_indices(robot_id)

    home_q = [0.0, -0.6, 0.0, -2.2, 0.0, 2.0, 0.8]
    reset_home(robot_id, arm_joints, finger_joints, home_q)

    cfg = PolicyConfig()
    result = run_fsm(robot_id, box_id, arm_joints, finger_joints, ee_link, cfg, recovery_demo=args.recovery_demo)
    print(f"\nFSM result: {result}")

    while True:
        step_sim(1)

if __name__ == "__main__":
    main()