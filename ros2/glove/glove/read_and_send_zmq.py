import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
import zmq
'''
This reads from websockets from Manus SDK and republishes to each glove topic

The joint level data is what Manus estimates your skeleton as in the order of thumb to pinky and MCP side, MCP forward, PIP, DIP.

The full skeleton is the xyz quaternion of every single joint in the hand estimated by Manus.  The short skeleton is just the minimum data required for IK, Tip and DIP locations.

Note, currently if you are using Windows, there is a different joint ordering than with Linux.  In Windows it is thumb to pinky in finger order, but on Linux it jumps around. Please see "short_idx" for details.  
'''

SKELETON_IP_ADDRESS = "tcp://192.168.1.97:8000"
ERGO_IP_ADDRESS = "tcp://192.168.1.97:8001"
LEFT_GLOVE_SN = "b8852964"
RIGHT_GLOVE_SN = "6bb82ce1"

# There are 25 skeletons per hand, 7 values per joint (x,y,z, quaternion), total 175 values per hand
# The first value is the serial number of the glove
skel_len = 176

# Ergonomics data has 20 values per hand
ergo_len = 20

# For computing inverse kinematics, we only need the fingertip and the previous joint xyz
# However, Windows and Linux have different joint orderings for the gloves (Windows in order)
# Linux: pinky, thumb, index, ring, middle
using_Linux = False
if using_Linux:
    short_idx = [23, 24, 4, 5, 9, 10, 19, 20, 14, 15]
else:
    # thumb PIP -> thumb DIP -> index ...
    short_idx = [3, 4, 8, 9, 13, 14, 18, 19, 23, 24]

# For debugging purposes, visualize the data as points instead of pose arrays


class GloveReader(Node):
    def __init__(self):
        super().__init__('glove_reader')
        # Connect to Server
        context_skel = zmq.Context()
        context_ergo = zmq.Context()
        self.socket_skel = context_skel.socket(zmq.PULL)
        self.socket_erog = context_ergo.socket(zmq.PULL)
        self.socket_skel.setsockopt(zmq.CONFLATE, True)
        self.socket_erog.setsockopt(zmq.CONFLATE, True)
        self.socket_skel.connect(SKELETON_IP_ADDRESS)
        self.socket_erog.connect(ERGO_IP_ADDRESS)
        self.pub_left = self.create_publisher(
            JointState, "/glove/l_joints", 10)
        self.pub_right = self.create_publisher(
            JointState, "/glove/r_joints", 10)

        self.pub_skeleton_right_full = self.create_publisher(
            PoseArray, '/glove/r_full', 1)
        self.pub_skeleton_right_full_markers = self.create_publisher(
            MarkerArray, '/glove/r_full_markers', 1
        )
        self.pub_skeleton_left_full = self.create_publisher(
            PoseArray, '/glove/l_full', 1)

        self.pub_skeleton_right_short = self.create_publisher(
            PoseArray, '/glove/r_short', 1)

        self.pub_skeleton_right_short_markers = self.create_publisher(
            MarkerArray, '/glove/r_short_markers', 1
        )

        self.pub_skeleton_left_short = self.create_publisher(
            PoseArray, '/glove/l_short', 1)
        # replace with your gloves (all lowercase letters)
        self.left_glove_sn = LEFT_GLOVE_SN
        self.right_glove_sn = RIGHT_GLOVE_SN

    # If you set a flag in the C++ code, you can send all the data that comes from the raw skeleton of the glove.
    # This data is from thumb to pinky, across all joints from palm to fingertip.   This can slow things down though

    def parse_full_skeleton_and_send(self, data):
        """
        Inside SDKClient.cpp function OnRawSkeletonStreamCallback,
        the flag full is set to true by default.
        """
        skeleton_list = []
        marker_array_msg = MarkerArray()

        for i in range(0, 25):
            # the first ID is right or left glove don't forget
            position = Point(
                x=float(data[1 + i*7]), y=float(data[2 + i*7]), z=float(data[3 + i*7]))
            orientation = Quaternion(x=float(
                data[4 + i*7]), y=float(data[5 + i*7]), z=float(data[6 + i*7]), w=float(data[7 + i*7]))
            # orientation = Quaternion(w=float(
            #     data[4 + i*7]), x=float(data[5 + i*7]), y=float(data[6 + i*7]), z=float(data[7 + i*7]))
            # orientation = Quaternion(
            #     x=float(0), y=float(0), z=float(0), w=float(0))
            # print("i: ", i, "position: ", position)
            pose = Pose(position=position, orientation=orientation)
            skeleton_list.append(pose)

            # Construct single sphere
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "joint_full"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.pose = pose
            marker_array_msg.markers.append(marker)

        output_array_msg = PoseArray()
        output_array_msg.header.frame_id = "map"
        output_array_msg.poses = skeleton_list

        if data[0] == self.left_glove_sn:
            self.pub_skeleton_left_full.publish(output_array_msg)
        elif data[0] == self.right_glove_sn:
            self.pub_skeleton_right_full.publish(output_array_msg)
            self.pub_skeleton_right_full_markers.publish(marker_array_msg)
        else:
            print("Glove serial number incorrect!")
            print(data[0])
    # This the dexcap style data, you only get the fingertip and the previous joint xyz as the data and then you can send that.  It goes from thumb_middle, thumb_tip, index_middle, index_tip etc.etc.

    def parse_short_skeleton_and_send(self, data):
        output_array_msg = PoseArray()
        output_array_msg.header.frame_id = "map"
        marker_array_msg = MarkerArray()
        for i in short_idx:
            # the first ID is right or left glove don't forget
            position = Point(
                x=float(data[1 + i*7]), y=float(data[2 + i*7]), z=float(data[3 + i*7]))
            # orientation = Quaternion(
            #     x=float(0), y=float(0), z=float(0), w=float(0))
            orientation = Quaternion(w=float(
                data[4 + i*7]), x=float(data[5 + i*7]), y=float(data[6 + i*7]), z=float(data[7 + i*7]))
            pose = Pose(position=position, orientation=orientation)
            output_array_msg.poses.append(pose)

            # Construct single sphere
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "joint_full"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose = pose
            marker_array_msg.markers.append(marker)

        if data[0] == self.left_glove_sn:
            self.pub_skeleton_left_short.publish(output_array_msg)
        elif data[0] == self.right_glove_sn:
            self.pub_skeleton_right_short.publish(output_array_msg)
            self.pub_skeleton_right_short_markers.publish(marker_array_msg)
        else:
            print("Glove serial number incorrect!")
            print(data[0])


def main(args=None):
    rclpy.init(args=args)
    glove_reader = GloveReader()
    while rclpy.ok():
        rclpy.spin_once(glove_reader, timeout_sec=0)
        # Read skeleton data first
        message_skel = glove_reader.socket_skel.recv()
        # receive the message_skel from the socket
        message_skel = message_skel.decode('utf-8')
        data_skel = message_skel.split(",")
        if data_skel is not None:
            try:
                # If full skeleton data two hands
                if len(data_skel) == skel_len * 2:
                    glove_reader.parse_full_skeleton_and_send(
                        data_skel[0:skel_len])
                    glove_reader.parse_full_skeleton_and_send(
                        data_skel[skel_len:skel_len * 2])
                    glove_reader.parse_short_skeleton_and_send(
                        data_skel[0:skel_len])
                    glove_reader.parse_short_skeleton_and_send(
                        data_skel[skel_len:skel_len * 2])
                # If full skeleton data_skel one hand
                elif len(data_skel) == skel_len:
                    glove_reader.parse_full_skeleton_and_send(
                        data_skel[0:skel_len])
                    glove_reader.parse_short_skeleton_and_send(
                        data_skel[0:skel_len])
            except KeyboardInterrupt as e:
                return
            except Exception as e:
                print("Exception reading skeleton data: ", e)

        # Read ergonomics data
        message_ergo = glove_reader.socket_erog.recv()
        message_ergo = message_ergo.decode('utf-8')
        data_ergo = message_ergo.split(",")
        # If joint level data
        if len(data_ergo) == ergo_len * 2:
            stater_msg = JointState()
            stater_msg.position = list(map(float, data_ergo[0:ergo_len]))
            glove_reader.pub_left.publish(stater_msg)
            stater_msg.position = list(
                map(float, data_ergo[ergo_len:ergo_len * 2]))
            glove_reader.pub_right.publish(stater_msg)
        else:
            print("Ergonomics data not in correct length: ", len(data_ergo))
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    glove_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
