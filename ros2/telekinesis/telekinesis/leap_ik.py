#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os

from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import JointState

'''
This takes the glove data, and runs inverse kinematics and then publishes onto LEAP Hand.

Note how the fingertip positions are matching, but the joint angles between the two hands are not.  :) 

Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''


class LeapPybulletIK(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')
        # start pybullet
        # clid = p.connect(p.SHARED_MEMORY)
        # clid = p.connect(p.DIRECT)
        p.connect(p.GUI)
        # load right leap hand
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        self.is_left = self.declare_parameter(
            'isLeft', False).get_parameter_value().bool_value
        print("Using left hand: ", self.is_left)
        # My hand 16 cm, leap hand 23 cm
        self.glove_to_leap_mapping_scale = 1.4375
        self.leapEndEffectorIndex = [3, 4, 8, 9, 13, 14, 18, 19]
        # self.leapEndEffectorIndex = [3, 4]

        if self.is_left:
            path_src = os.path.join(
                path_src, "leap_hand_mesh_left/robot_pybullet.urdf")
            position = [-0.05, -0.03, -0.25]
            sub_topic = "/glove/l_short"
            pub_topic = "/leaphand_node/cmd_allegro_left"
        else:
            path_src = os.path.join(
                path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
            position = [-0.05, -0.03, -0.125]
            sub_topic = "/glove/r_short"
            pub_topic = "/leaphand_node/cmd_allegro_right"

        self.pub_hand = self.create_publisher(
            JointState, pub_topic, 10)
        self.sub_skeleton = self.create_subscription(
            PoseArray, sub_topic, self.glove_callback, 10)

        self.LeapId = p.loadURDF(
            path_src,
            position,
            p.getQuaternionFromEuler([0, 1.57, 1.57]),
            useFixedBase=True
        )

        print("Leap hand ID: ", self.LeapId)
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
        Subscribe to glove skeleton data (8 entries per message).
        Perform scaling and offsetting to match LEAP hand dimension.
        """
        glove_poses = pose_array.poses
        leap_pos = []

        glove_pose: Pose
        for glove_pose in glove_poses:
            leap_x = glove_pose.position.x * self.glove_to_leap_mapping_scale
            leap_y = glove_pose.position.y * self.glove_to_leap_mapping_scale
            leap_z = -glove_pose.position.z * self.glove_to_leap_mapping_scale
            leap_pos.append([leap_x, leap_y, leap_z])

        # this isn't great because they won't oppose properly
        # leap_pos[2][0] = leap_pos[2][0] - 0.02
        # leap_pos[3][0] = leap_pos[3][0] - 0.02
        # leap_pos[6][0] = leap_pos[6][0] + 0.02
        # leap_pos[7][0] = leap_pos[7][0] + 0.02
        # leap_pos[2][1] = leap_pos[2][1] + 0.002
        # leap_pos[4][1] = leap_pos[4][1] + 0.002
        # leap_pos[6][1] = leap_pos[6][1] + 0.002
        self.compute_IK(leap_pos)
        self.update_target_vis(leap_pos)

    def compute_IK(self, leap_pos):
        p.stepSimulation()

        rightHandThumb_middle_pos = leap_pos[0]
        rightHandThumb_pos = leap_pos[1]

        rightHandIndex_middle_pos = leap_pos[2]
        rightHandIndex_pos = leap_pos[3]

        rightHandMiddle_middle_pos = leap_pos[4]
        rightHandMiddle_pos = leap_pos[5]

        rightHandRing_middle_pos = leap_pos[6]
        rightHandRing_pos = leap_pos[7]

        leapEndEffectorPos = [
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandThumb_middle_pos,
            rightHandThumb_pos,
        ]

        """
        targetPositions: target position of the end effector 
        (its link coordinate, not center of mass coordinate!). 
        By default this is in Cartesian world space, 
        unless you provide currentPosition joint angles.
        """
        jointPoses = p.calculateInverseKinematics2(
            bodyUniqueId=self.LeapId,
            endEffectorLinkIndices=self.leapEndEffectorIndex,
            targetPositions=leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )

        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (
            0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints
        for i in range(20):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        real_robot_hand_q = np.array([float(0.0) for _ in range(16)])
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
