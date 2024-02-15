#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import actionlib
from a1_chatgpt_demo.msg import WaypointAction, WaypointGoal, WaypointResult


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

def bounding(x, min, max):
    if x < min:
        x = min
    if x > max:
        x = max
    return x

def angleBounding(x):
    if x > 1.5 * np.pi:
        x = x - 2 * np.pi
    if x < -1.5 * np.pi:
        x = x + 2 * np.pi
    return x

class WaypointPending(object):

    def __init__(self):

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber("/torso_odom", Odometry, self.callback_odom)
        self.stand_srv = rospy.ServiceProxy("/standing", Empty)
        self.walk_srv = rospy.ServiceProxy("/walking", Empty)
        
        # self.gpt_sub = rospy.Subscriber('/gpt_waypoint', Float64MultiArray, self.gpt_callback)
        self.gpt_action_server = actionlib.SimpleActionServer(
            '/gpt_waypoint_pending', WaypointAction, execute_cb=self.gpt_callback, auto_start=False
        )

        # (x, y, theta)
        self.waypoint_pose = None
        self.cur_pos = np.array([0.0, 0.0, 0.0])
        self.rot_mat = np.zeros((3, 3))

        self.move_flag = False
        self.moving_mode = "initial"
        self.cur_vel = 0.0
        self.prev_vel = 0.0
        self.count = 0

        self.control_msg = Twist()
        self.stop_msg = Twist()
        self.result = None 

        self.r = rospy.Rate(1)

        self.gpt_action_server.start()

    def gpt_callback(self, goal):

        self.move_flag = True
        self.moving_mode = "initial"

        self.waypoint_pose = np.array(goal.pose)

        self.result = WaypointResult()
        
        while self.moving_mode != "stop":

            if self.gpt_action_server.is_preempt_requested():
                self.gpt_action_server.set_preempted()

            self.r.sleep()

        self.result.success = True
        
        print("[Waypoint Pending] Waypoint reached")
        self.gpt_action_server.set_succeeded(self.result)        

    def callback_odom(self, msg):

        # print("callback_odom")

        if self.move_flag is False:
            return

        self.cur_pos[0] = msg.pose.pose.position.x
        self.cur_pos[1] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.cur_pos[2] = yaw

        self.rot_mat = calcHomogeneousMatrix(self.cur_pos[0], self.cur_pos[1], self.cur_pos[2])
        self.move()

    def move(self):

        if self.move_flag is False:
            return

        # World frame Diff
        diff_pose = self.waypoint_pose - self.cur_pos
        diff_pose[2] = angleBounding(diff_pose[2])

        # World frame angle
        diff_angle = np.arctan2(diff_pose[1], diff_pose[0])
        # diff_angle = angleBounding(diff_angle)

        # Robot frame angle
        target_angle_diff = diff_angle - self.cur_pos[2]

        # vel command body frame
        diff_pose_robot = self.rot_mat.T @ diff_pose

        # Initial Start
        if self.moving_mode == "initial":
            self.control_msg.linear.x = 0.0
            self.control_msg.linear.y = 0.0
            self.control_msg.angular.z = 0.0
            
            res = self.walk_srv()
            self.moving_mode = "first turn"
            print("[Waypoint Pending] Start Moving to the waypoint")

        # Turn first
        if self.moving_mode == "first turn" and np.abs(target_angle_diff) > 0.1:
            self.control_msg.linear.x = 0.0
            self.control_msg.linear.y = 0.0
            omega = bounding(0.8 * target_angle_diff, -1.0, 1.0)
            self.control_msg.angular.z = omega
            # print("[first turn] target_angle_diff: ", target_angle_diff)
        if self.moving_mode == "first turn" and np.abs(target_angle_diff) < 0.1:
            self.control_msg.angular.z = 0.0
            self.moving_mode = "moving forward"
            print("[Waypoint Pending] First turn finished")

        # Moving Forward
        if self.moving_mode == "moving forward":
            xvel = bounding( diff_pose_robot[0], -0.6, 0.6)
            yvel = bounding( diff_pose_robot[1], -0.6, 0.6)
            
            # 갑자기 빨라지는 것 방지
            self.cur_vel = np.linalg.norm([xvel, yvel])
            
            if np.abs(self.cur_vel - self.prev_vel) > 0.3 and self.count < 5000:
                xvel /= 2
                yvel /= 2
                self.count += 1
                # print("[moving forward] vel limit")
            if self.count == 5000:
                self.prev_vel = self.cur_vel
                self.count = 0

            self.control_msg.linear.x = xvel
            self.control_msg.linear.y = yvel
            self.control_msg.angular.z = 0.0
            # print("[moving forward] diff X / diff Y : ", diff_pose_robot[0], diff_pose_robot[1])
        if self.moving_mode == "moving forward" and np.linalg.norm(diff_pose[:-1]) < 0.1:
            self.control_msg.linear.x = 0.0
            self.control_msg.linear.y = 0.0
            self.control_msg.angular.z = 0.0

            self.moving_mode = "second turn"
            print("[Waypoint Pending] Moving forward finished")

        # Turn AgainTrueturn" and np.abs(diff_pose[2]) < 0.05:
        if self.moving_mode == "second turn" and np.abs(diff_pose[2]) > 0.05:
            xvel = bounding( diff_pose_robot[0], -0.6, 0.6)
            yvel = bounding( diff_pose_robot[1], -0.6, 0.6)            
            self.control_msg.linear.x = xvel
            self.control_msg.linear.y = yvel

            omega = bounding(0.8 * diff_pose[2], -1.0, 1.0)
            self.control_msg.angular.z = omega
            # print("[second turn] diff_pose : ", diff_pose_robot[2])

        if self.moving_mode == "second turn" and np.abs(diff_pose[2]) < 0.05:
            self.control_msg.angular.z = 0.0
            self.moving_mode = "stop"
            print("[Waypoint Pending] Second turn finished")
            res = self.stand_srv()
            self.move_flag = False

        self.pub.publish(self.control_msg)

def target_waypoint():

    rospy.init_node("waypoint_control_pending_node", anonymous=True)
    waypoint_control = WaypointPending()

    rospy.spin()

if __name__ == "__main__":

    try:
        target_waypoint()
    except KeyboardInterrupt:
        print("Shutting down")