#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#reference: lnotspotl

# TODO : roll, pitch(-0.3 ~ 0.3), yaw subscribe from other topic

import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import MotorCmd, LowCmd
from InverseKinematics import robot_IK

class RobotController:
    def __init__(self):
        self.inverseKinematics = None
        self.publishers = []
        self.low_cmd = LowCmd()
        
        # 몸체 가로 / 세로
        self.body = [0.366, 0.094]
        # 각 다리 link 길이
        self.legs = [0., 0.08505, 0.2, 0.2]
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.command_topics = [
            "/a1_gazebo/FR_hip_controller/command", "/a1_gazebo/FR_thigh_controller/command",
            "/a1_gazebo/FR_calf_controller/command", "/a1_gazebo/FL_hip_controller/command",
            "/a1_gazebo/FL_thigh_controller/command", "/a1_gazebo/FL_calf_controller/command",
            "/a1_gazebo/RR_hip_controller/command", "/a1_gazebo/RR_thigh_controller/command",
            "/a1_gazebo/RR_calf_controller/command", "/a1_gazebo/RL_hip_controller/command",
            "/a1_gazebo/RL_thigh_controller/command", "/a1_gazebo/RL_calf_controller/command"
        ]
        self.RATE = 60

        for i in range(len(self.command_topics)):
            self.publishers.append(rospy.Publisher(self.command_topics[i], MotorCmd, queue_size=0))
        self.pitch_sub = rospy.Subscriber("/pitch_control", Float64, self.pitch_callback)

        self.setup_controller()

    def pitch_callback(self, data):
        
        self.pitch = data.data

        # rospy.loginfo("Received float value: %f", self.pitch)

    def setup_controller(self):
        self.inverseKinematics = robot_IK.InverseKinematics(self.body, self.legs)

        for i in range(4):
            self.low_cmd.motorCmd[i*3+0].mode = 0x0A
            self.low_cmd.motorCmd[i*3+0].Kp = 70
            self.low_cmd.motorCmd[i*3+0].dq = 0
            self.low_cmd.motorCmd[i*3+0].Kd = 3
            self.low_cmd.motorCmd[i*3+0].tau = 0

            self.low_cmd.motorCmd[i*3+1].mode = 0x0A
            self.low_cmd.motorCmd[i*3+1].Kp = 180
            self.low_cmd.motorCmd[i*3+1].dq = 0
            self.low_cmd.motorCmd[i*3+1].Kd = 8
            self.low_cmd.motorCmd[i*3+1].tau = 0

            self.low_cmd.motorCmd[i*3+2].mode = 0x0A
            self.low_cmd.motorCmd[i*3+2].Kp = 300
            self.low_cmd.motorCmd[i*3+2].dq = 0
            self.low_cmd.motorCmd[i*3+2].Kd = 15
            self.low_cmd.motorCmd[i*3+2].tau = 0

    def run(self):
    
        dx, dy, dz = 0.0, 0.0, 0.0
        roll, pitch, yaw = self.roll, self.pitch, self.yaw

        leg_positions = np.array([
            [0.203, 0.203, -0.183, -0.183],
            [-0.13205, 0.13205, -0.13205, 0.13205],
            [-0.3, -0.3, -0.3, -0.3]
        ])

        try:
            joint_angles = self.inverseKinematics.inverse_kinematics(leg_positions, dx, dy, dz, roll, pitch, yaw)
            new_joint_angles = joint_angles[3:6] + joint_angles[0:3] + joint_angles[9:12] + joint_angles[6:9]

            for i in range(len(new_joint_angles)):
                self.low_cmd.motorCmd[i].q = new_joint_angles[i]
                self.publishers[i].publish(self.low_cmd.motorCmd[i])

        except Exception as e:
            print("An error occurred:", e)

if __name__ == "__main__":

    rospy.init_node("rpy_controller")

    controller = RobotController()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        controller.run()
        rate.sleep()
