import pybullet as p
import time

def step_sim(n=240):
    for i in range(n):
        p.stepSimulation()
        time.sleep(1.0/240.0)


def open_gripper(robot_id, finger_joints, open_width=0.04, force=50):
    """Function to open gripper"""

    for fj in finger_joints:
        p.setJointMotorControl2(robot_id, fj, p.POSITION_CONTROL, open_width, force=force)


def close_gripper(robot_id, finger_joints, close_width=0.0, force=50):
    """Function to close gripper"""

    for fj in finger_joints:
        p.setJointMotorControl2(robot_id, fj, p.POSITION_CONTROL, close_width, force=force)


def move_arm_joints(robot_id, arm_joints, q, force=200):
    for k, j in enumerate(arm_joints):
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=q[k], force=force)


def move_ee_pose(robot_id, ee_link, arm_joints, target_pos, target_orientation, steps=240, force=200):
    """IK calculator for the manipulator"""

    q = p.calculateInverseKinematics(
        robot_id,
        ee_link,
        target_pos,
        target_orientation,
        maxNumIterations=200,
        residualThreshold=1e-4
    )

    move_arm_joints(robot_id, arm_joints, q, force)
    step_sim(steps)