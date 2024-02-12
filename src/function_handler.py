#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class GPTFunctionHandler(object):

    def __init__(self):

        self.standing_pub = rospy.Publisher('/gpt_standing', EmptyMsg, queue_size=1)
        self.picture_pub = rospy.Publisher('/gpt_pictrue', EmptyMsg, queue_size=1)
        self.video_pub = rospy.Publisher('/gpt_video', Float64, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/gpt_waypoint', Float64MultiArray, queue_size=1)
        self.pitch_ctrl_pub = rospy.Publisher('/gpt_pitch_control', Float64MultiArray, queue_size=1)

        self.cur_pose_pub = rospy.Publisher('/gpt_cur_pose', EmptyMsg, queue_size=1)

    def take_picture(self):
        self.picture_pub.publish(EmptyMsg())
        rospy.loginfo("Take Picture!")

    def e_stop(self):
        self.standing_pub.publish(EmptyMsg())
        rospy.loginfo("Emergency Stop!")

    def shoot_video(self, seconds):
        msg = Float64()
        msg.data = seconds
        self.video_pub.publish(msg)
        rospy.loginfo("Shoot Video!")

    def target_waypoint(self, x, y, theta):
        array_data = [x, y, theta]  # Example array with three elements
        self.waypoint_pub.publish(Float64MultiArray(data=array_data))
        rospy.loginfo("Target Waypoint!")

    def pitch_control(self, min_angle, max_angle):
        array_data = [min_angle, max_angle]  # Example array with three elements
        self.pitch_ctrl_pub.publish(Float64MultiArray(data=array_data))
        rospy.loginfo("Pitch Control!")

    def get_cur_pose(self):
        self.cur_pose_pub.publish(EmptyMsg())
        rospy.loginfo("Get Current Pose!")