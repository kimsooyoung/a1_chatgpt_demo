#! /usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class PitchController:
    def __init__(self):
        self.publisher = rospy.Publisher("/pitch_control", Float64, queue_size=10)
        self.gpt_sub = rospy.Subscriber('/gpt_pitch_control', Float64MultiArray, self.callback)
        
        self.rate = rospy.Rate(20)

    def callback(self, msg):
        self.min_angle, self.max_angle = msg.data

        pitch_list_1 = np.linspace(0.0, self.min_angle, 10)
        pitch_list_2 = np.linspace(self.min_angle, self.max_angle, 20)
        pitch_list_3 = np.linspace(self.max_angle, 0.0, 10)

        # pitch_list = pitch_list_1 + pitch_list_2 + pitch_list_3
        pitch_list = np.concatenate((pitch_list_1, pitch_list_2, pitch_list_3), axis=0)

        for pitch in pitch_list:
            self.pitch_publish(pitch)
            self.rate.sleep()

    def pitch_publish(self, data):
        self.publisher.publish(data)

def pitch_control():
    
    rospy.init_node("pitch_controller")
    publisher = PitchController()

    rospy.spin()

if __name__ == "__main__":
    
    try:
        pitch_control()
    except KeyboardInterrupt:
        print("Shutting down")
