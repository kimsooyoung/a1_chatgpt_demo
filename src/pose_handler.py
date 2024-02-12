#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def calcHomogeneousMatrix(x, y, theta):

    output = np.zeros((3, 3))

    output[0, 0] = np.cos(theta)
    output[0, 1] = -np.sin(theta)
    output[0, 2] = x

    output[1, 0] = np.sin(theta)
    output[1, 1] = np.cos(theta)
    output[1, 2] = y

    output[2, 0] = 0
    output[2, 1] = 0
    output[2, 2] = 1

    return output

class CurrentPose(object):

    def __init__(self):

        self.sub_odom = rospy.Subscriber("/torso_odom", Odometry, self.odom_callback)
        self.gpt_sub = rospy.Subscriber('/gpt_cur_pose', EmptyMsg, self.gpt_callback)
        
        # (x, y, theta)
        self.waypoint_pose = None
        self.cur_pos = np.array([0.0, 0.0, 0.0])
        self.rot_mat = np.zeros((3, 3))

        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

    def gpt_callback(self, msg):

        print(f"X : {self.cur_pos[0]}, Y : {self.cur_pos[1]}, Theta : {self.cur_pos[2]}")
        print(f"Roll : {self.roll}, Pitch : {self.pitch}, Yaw : {self.yaw}")

    def odom_callback(self, msg):

        self.cur_pos[0] = msg.pose.pose.position.x
        self.cur_pos[1] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.cur_pos[2] = self.yaw

        self.rot_mat = calcHomogeneousMatrix(self.cur_pos[0], self.cur_pos[1], self.cur_pos[2])

def get_cur_pose():

    rospy.init_node("cur_pose_node")
    cur_pose_node = CurrentPose()

    rospy.spin()

if __name__ == "__main__":
    
    try:
        get_cur_pose()
    except KeyboardInterrupt:
        print("Shutting down")
