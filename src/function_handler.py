#! /usr/bin/env python3

import time
import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from a1_chatgpt_demo.msg import WaypointAction, WaypointGoal

class GPTFunctionHandler(object):

    def __init__(self):

        self.standing_pub = rospy.Publisher('/gpt_standing', EmptyMsg, queue_size=1)
        self.walking_pub = rospy.Publisher('/gpt_walking', EmptyMsg, queue_size=1)
        self.picture_pub = rospy.Publisher('/gpt_pictrue', EmptyMsg, queue_size=1)
        self.video_pub = rospy.Publisher('/gpt_video', Float64, queue_size=1)
        self.waypoint_pub = rospy.Publisher('/gpt_waypoint', Float64MultiArray, queue_size=1)
        self.pitch_ctrl_pub = rospy.Publisher('/gpt_pitch_control', Float64MultiArray, queue_size=1)
        self.relative_pos_pub = rospy.Publisher('/gpt_relative_position', Float64MultiArray, queue_size=1)
        
        # self.cur_pose_pub = rospy.Publisher(, EmptyMsg, queue_size=1)
        self.cur_pose_client = rospy.ServiceProxy('/gpt_cur_pose', Trigger)
        self.waypoint_action_server = actionlib.SimpleActionClient('/gpt_waypoint_pending', WaypointAction)
        self.pos_action_server = actionlib.SimpleActionClient('/gpt_relative_position_pending', WaypointAction)

    def take_picture(self):
        self.picture_pub.publish(EmptyMsg())
        rospy.loginfo("Take Picture!")

    def walking_again(self):
        self.walking_pub.publish(EmptyMsg())
        rospy.loginfo("Walking Again!")

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
        res = self.cur_pose_client(TriggerRequest())
        rospy.loginfo(res.message)
        return res.success

    def relative_pos_control(self, x, y, theta):
        array_data = [x, y, theta]
        self.relative_pos_pub.publish(Float64MultiArray(data=array_data))
        rospy.loginfo("Relative Position Control!")

    def target_waypoint_pending(self, x, y, theta):
        waypoint_goal = WaypointGoal()
        waypoint_goal.pose = [x, y, theta]

        self.waypoint_action_server.send_goal(waypoint_goal)
        self.waypoint_action_server.wait_for_result()

        succeed = False
        print("Waiting for waypoint to be reached...")

        result = self.waypoint_action_server.get_result()
        succeed = result.success
        
        if succeed:
            rospy.loginfo("Waypoint reached!")

    def relative_pos_control_pending(self, x, y, theta):
        waypoint_goal = WaypointGoal()
        waypoint_goal.pose = [x, y, theta]

        self.pos_action_server.send_goal(waypoint_goal)
        self.pos_action_server.wait_for_result()

        succeed = False
        print("Waiting for relative position to be reached...")

        result = self.pos_action_server.get_result()
        succeed = result.success
        
        if succeed:
            rospy.loginfo("Position reached!")
