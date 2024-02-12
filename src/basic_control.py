#! /usr/bin/env python3

# TODO: sub Odom and get the current position of the robot 
# TODO : capture function

import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry


class BasicControl(object):

    def __init__(self):

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber("/torso_odom", Odometry, self.callback_odom)

        self.r = rospy.Rate(1)

        self.odom = Odometry()
        self.pose = Pose()

        self.control_msg = Twist()
        self.control_time = 0.0

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.0
        self.stop_msg.linear.y = 0.0
        self.stop_msg.angular.z = 0.0

        rospy.loginfo("==== Basic Control node Started Type Your Command ====\n")

    def callback_odom(self, msg):
        self.odom = msg
        self.pose = self.odom.pose.pose

    def move(self):

        start_time = time.time()

        rospy.loginfo("Moving Now... Takes %s seconds", self.control_time)

        while time.time() - start_time < self.control_time:
            self.pub.publish(self.control_msg)

        self.pub.publish(self.stop_msg)

    def move_forward(self, meter=0.0):
        self.control_msg.linear.x = 0.5
        self.control_msg.angular.z = 0.0
        self.control_time = meter / 0.5

        self.move()

    def move_backward(self, meter=0.0):
        self.control_msg.linear.x = -0.5
        self.control_msg.angular.z = 0.0
        self.control_time  = meter / 0.5

        self.move()

    def turn_left(self, degree=0.0):
        self.control_msg.linear.x = 0.0
        self.control_msg.angular.z = 0.5
        self.control_time = degree / 0.5

        self.move()

    def turn_right(self, degree=0.0):
        self.control_msg.linear.x = 0.0
        self.control_msg.angular.z = -0.5
        self.control_time = degree / 0.5

        self.move()

if __name__ == "__main__":
    
    rospy.init_node("basic_control_node")

    basic_control = BasicControl()
    
    while not rospy.is_shutdown():

        str_input = input("Type Your Command: ")
        str_input = str_input.lower()
        rospy.loginfo("Your Command: %s", str_input)

        if "exit" in str_input:
            rospy.loginfo("Exit the Basic Control node")
            break

        splited_str = str_input.split()

        # Parse Message
        if "move forward" in str_input:
            moving_number = float(splited_str[-2])
            moving_unit = splited_str[-1]
            if moving_unit == "cm" or moving_unit == "centimeter":
                moving_number = moving_number / 100
            basic_control.move_forward(moving_number)
        elif "move backward" in str_input:
            moving_number = float(splited_str[-2])
            moving_unit = splited_str[-1]
            if moving_unit == "cm" or moving_unit == "centimeter":
                moving_number = moving_number / 100
            basic_control.move_backward(moving_number)
        elif "turn left" in str_input:
            moving_number = float(splited_str[-2])
            moving_unit = splited_str[-1]
            if moving_unit == "degree" or moving_unit == "degrees":
                moving_number = moving_number / 180 * 3.141592
            basic_control.turn_left(moving_number)
        elif "turn right" in str_input:
            moving_number = float(splited_str[-2])
            moving_unit = splited_str[-1]
            if moving_unit == "degree" or moving_unit == "degrees":
                moving_number = moving_number / 180 * 3.141592
            basic_control.turn_right(moving_number)
