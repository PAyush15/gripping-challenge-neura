import pybullet as p
import pybullet_data

def setup_sim(gui: bool = True):
    cid = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    return cid

def load_world(table_pos=(0.9, 0.0, 0.0), box_pos=(0.3, 0.0, 0.65)):
    plane_id = p.loadURDF("plane.urdf")
    table_id = p.loadURDF("table/table.urdf", basePosition=list(table_pos))
    box_id = p.loadURDF("cube_small.urdf", basePosition=list(box_pos))

    return plane_id, table_id, box_id

def load_panda(base_pos=(0.0, 0.0, 0.0), fixed_base=True):
    robot_id = p.loadURDF("franka_panda/panda.urdf",
                        basePosition=[0.0, 0.0, 0.0],
                        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=fixed_base)
    return robot_id

def get_panda_indices(robot_id: int):
    """Parse joint indices numbers"""

    arm_joints = []
    finger_joints = []
    ee_link = None

    n = p.getNumJoints(robot_id)
    for i in range(n):
        info = p.getJointInfo(robot_id, i)
        jname = info[1].decode("utf-8")
        lname = info[12].decode("utf-8")

        if jname.startswith("panda_joint") and info[2] == p.JOINT_REVOLUTE:
            arm_joints.append(i)

        if "finger_joint" in jname:
            finger_joints.append(i)

        if lname == "panda_grasptarget":
            ee_link = i

    arm_joints = sorted(arm_joints)
    finger_joints = sorted(finger_joints)

    if ee_link is None:
        print("Warning: EE link not found")
        ee_link = arm_joints[-1]

    return arm_joints, finger_joints, ee_link

def reset_home(robot_id: int, arm_joints, finger_joints, home_q):
    for j,q in zip(arm_joints, home_q):
        p.resetJointState(robot_id, j, q)

    for fj in finger_joints:
        p.resetJointState(robot_id, fj, 0.04)
    