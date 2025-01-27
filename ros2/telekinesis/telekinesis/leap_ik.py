#!/usr/bin/env python3
import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState

# The ratio of LEAP hand to glove hand (human hand) dimensions
GLOVE_to_LEAP_FACTOR = 1.4375
# GLOVE_to_LEAP_FACTOR = 1.0
GLOVE_to_LEAP_FACTOR = 1.8


# PIP and Fingertip indices (should match with the actual)
LEAP_EE_URDF_INDEX_CHECK = [14, 15, 2, 3, 6, 7, 10, 11]

# Create mapping from link name to link index
LINK_MAP = {}

# Maintain same link order everytime?
MAINTAIN_LINK_ORDER = True

# Thumb (mid, tip), Index (mid, tip), Middle (mid, tip), Ring (mid, tip) index from full skeleton
FINGER_INDEX = [3, 4, 8, 9, 13, 14, 18, 19, 23, 24]

# Glove to Pybullet (glove palm down, pybullet palm up)
g_T_p = np.eye(4)
g_T_p = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0],
                 [0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

# g_T_p = np.array([[0.9999974,  0.0015927,  0.0015927, 0.0],
#                   [0.0015927, -0.9999987,  0.0000000, 0.0],
#                   [0.0015927,  0.0000025, -0.9999987, 0.0],
#                   [0.0, 0.0, 0.0, 1.0]])


def ros_pose_to_mat(pose: Pose):
    """Convert geometry/Pose into np.array 4x4 matrix"""
    pose_mat = np.eye(4)
    # Scalar (w) first = False
    R = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w]).as_matrix()
    pose_mat[:3, :3] = R
    pose_mat[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return pose_mat


class LeapPybulletIK(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')
        # start pybullet
        p.connect(p.GUI)
        # load right leap hand
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        self.is_left = self.declare_parameter(
            'isLeft', False).get_parameter_value().bool_value
        print("Using left hand: ", self.is_left)

        # Since the root of the LEAP hand URDF is not at the palm's root (it is at the root of the index finger), we set an offset to correct the root location
        self.leap_center_offset = [0.18, 0.03, 0.0]
        # Since the root of the fingertip mesh in URDF is not at the tip (it is at the right lower part of the fingertip mesh), we set an offset to correct the fingertip location
        self.fingertip_offset = np.array([0.1, 0.0, -0.08])
        # Same reason for the thumb tip
        self.thumb_offset = np.array([0.1, 0.0, -0.06])

        self.position_offset = [0.10, -0.05, 0.0]

        if self.is_left:
            path_src = os.path.join(
                path_src, "leap_hand_mesh_left/robot_pybullet.urdf")
            # orientation = p.getQuaternionFromEuler([0, 1.57, 1.57])
            orientation = p.getQuaternionFromEuler([0, 0, 0])
            sub_topic = "/glove/l_full"
            pub_topic = "/leaphand_node/cmd_allegro_left"
        else:
            path_src = os.path.join(
                path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
            # In pybullet red is X, green is Y, blue is Z
            # position = [0.0, 0.0, 0.0]
            orientation = p.getQuaternionFromEuler([0.0, 3.14, 3.14])
            sub_topic = "/glove/r_full"
            pub_topic = "/leaphand_node/cmd_allegro_right"

        self.pub_hand = self.create_publisher(
            JointState, pub_topic, 10)
        self.sub_skeleton = self.create_subscription(
            PoseArray, sub_topic, self.glove_callback, 10)

        if MAINTAIN_LINK_ORDER:
            self.LeapId = p.loadURDF(
                path_src,
                self.position_offset,
                orientation,
                useFixedBase=True,
                flags=p.URDF_MAINTAIN_LINK_ORDER
            )
        else:
            self.LeapId = p.loadURDF(
                path_src,
                self.position_offset,
                orientation,
                useFixedBase=True
            )
        print("Leap hand ID: ", self.LeapId)

        # Get the name and type info for a joint on a body
        self.numJoints = p.getNumJoints(self.LeapId)
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.LeapId, i)
            link_idx = jointInfo[0]
            jointName = jointInfo[1].decode('utf-8')
            childLinkName = jointInfo[12].decode('utf-8')
            print("link_idx:", link_idx, " jointName: ",
                  jointName, " childLinkName: ", childLinkName)
            LINK_MAP[childLinkName] = link_idx

        self.LEAP_EE_URDF_INDEX = [LINK_MAP["thumb_dip"], LINK_MAP["thumb_fingertip"],
                                   LINK_MAP["dip"], LINK_MAP["fingertip"],
                                   LINK_MAP["dip_2"], LINK_MAP["fingertip_2"],
                                   LINK_MAP["dip_3"], LINK_MAP["fingertip_3"]]

        print("Calculated LEAP_EE_INDEX: ", self.LEAP_EE_URDF_INDEX)
        print("LEAP_EE_URDF_INDEX_CHECK: ", LEAP_EE_URDF_INDEX_CHECK)
        assert self.LEAP_EE_URDF_INDEX == LEAP_EE_URDF_INDEX_CHECK

        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()

    def create_target_vis(self):
        # load balls
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]

        self.ballMbt = []
        for i in range(0, 4):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape,
                                basePosition=basePosition))  # for base and finger tip joints
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(
                self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1])
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1])
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])

    def update_target_vis(self, leap_pos):
        _, current_orientation = p.getBasePositionAndOrientation(
            self.ballMbt[0])
        p.resetBasePositionAndOrientation(
            self.ballMbt[0], leap_pos[0], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(
            self.ballMbt[1])
        p.resetBasePositionAndOrientation(
            self.ballMbt[1], leap_pos[2], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(
            self.ballMbt[2])
        p.resetBasePositionAndOrientation(
            self.ballMbt[2], leap_pos[4], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(
            self.ballMbt[3])
        p.resetBasePositionAndOrientation(
            self.ballMbt[3], leap_pos[6], current_orientation)

    def glove_callback(self, pose_array: PoseArray):
        """
        Subscribe to the full skeleton pose array.
        The first pose is suppose to be at the origin.
        The following 8 poses form the palm plane.
        Scale them accordingly. Now the joint angles between human hand and leap hand are supposed to be the same.
        Scale the lenear components of the poses by the GLOVE_to_LEAP_FACTOR.
        """
        glove_poses = pose_array.poses
        leap_pos = []

        glove_pose: Pose
        N = len(glove_poses)  # number of mocap joints
        # Origin pose to world
        w_T_o: np.ndarray = ros_pose_to_mat(glove_poses[0])
        print("W_T_o: ", w_T_o)
        # Sanity check if translation component is zero
        assert w_T_o[0, 3] == w_T_o[1, 3] == w_T_o[2, 3] == 0.0
        o_T_w = np.linalg.inv(w_T_o)

        # Transformed poses to joint 0 frame
        o_Ts = np.zeros((N, 4, 4))

        for (i, glove_pose) in enumerate(glove_poses):
            # Find origin pose (assume T = 0 and only R is non-identity)
            # pose in joint 0 frame (4x4) matrix
            o_T_i = g_T_p @ o_T_w @ ros_pose_to_mat(glove_pose)
            o_Ts[i] = o_T_i

        leap_pos = self.lin_scale_ee(o_Ts, GLOVE_to_LEAP_FACTOR)
        self.compute_IK(leap_pos)
        self.update_target_vis(leap_pos)

    def lin_scale_ee(self, oTs: np.ndarray, scale: float):
        """
        Scale the linear components of the end effector poses by the GLOVE_to_LEAP_FACTOR.
        Order should follow mid->tip with thumb, index, middle, ring
        oTs: Nx4x4 array of end effector poses in joint 0 frame
        """
        leap_pos = []
        for i in FINGER_INDEX:
            o_T_i = oTs[i]
            # Scale the linear components
            scaled_t = scale * o_T_i[0:3, 3]
            # scaled_t += (self.position_offset)
            leap_pos.append(scaled_t)
        return leap_pos

    def compute_IK(self, leap_pos):
        p.stepSimulation()

        Thumb_middle_pos = leap_pos[0]
        Thumb_pos = leap_pos[1]
        print("thumb_pos: ", Thumb_pos)

        Index_middle_pos = leap_pos[2]
        Index_pos = leap_pos[3]
        print("index_pos: ", Index_pos)

        Middle_middle_pos = leap_pos[4]
        Middle_pos = leap_pos[5]
        print("middle_pos: ", Middle_pos)

        Ring_middle_pos = leap_pos[6]
        Ring_pos = leap_pos[7]
        print("ring_pos: ", Ring_pos)

        leapEndEffectorPos = [
            Thumb_middle_pos,
            Thumb_pos,
            Index_middle_pos,
            Index_pos,
            Middle_middle_pos,
            Middle_pos,
            Ring_middle_pos,
            Ring_pos,
        ]

        """
        targetPositions: target position of the end effector 
        (its link coordinate, not center of mass coordinate!). 
        By default this is in Cartesian world space, 
        unless you provide currentPosition joint angles.
        """
        jointPoses = p.calculateInverseKinematics2(
            bodyUniqueId=self.LeapId,
            endEffectorLinkIndices=self.LEAP_EE_URDF_INDEX,
            targetPositions=leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )

        # Update the hand joints
        for i in range(self.numJoints):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        real_robot_hand_q = np.array([float(0.0)
                                     for _ in range(self.numJoints)])
        # real_left_robot_hand_q = np.array([0.0 for _ in range(16)])

        real_robot_hand_q[0:4] = jointPoses[0:4]
        real_robot_hand_q[4:8] = jointPoses[4:8]
        real_robot_hand_q[8:12] = jointPoses[8:12]
        real_robot_hand_q[12:16] = jointPoses[12:16]
        real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        stater = JointState()
        stater.position = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(stater)


def main(args=None):
    rclpy.init(args=args)
    leappybulletik = LeapPybulletIK()
    rclpy.spin(leappybulletik)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leappybulletik.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
