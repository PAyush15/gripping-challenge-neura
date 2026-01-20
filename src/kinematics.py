import pybullet as p
import math

def down_orientation():
    return p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

def get_object_pose(body_id: int):
    return p.getBasePositionAndOrientation(body_id)

def compute_grasp_poses(box_id: int, grasp_z_offset=0.03):
    (ox, oy, oz), _ = get_object_pose(box_id)
    pregrasp = [ox, oy, oz + 0.20]
    grasp    = [ox, oy, oz + grasp_z_offset]
    lift     = [ox, oy, oz + 0.30]
    
    return pregrasp, grasp, lift