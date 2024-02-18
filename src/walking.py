#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse


class Walking(object):

    def __init__(self):

        self.walk_srv = rospy.ServiceProxy("/walking", Empty)
        self.gpt_sub = rospy.Subscriber('/gpt_walking', EmptyMsg, self.callback)

    def callback(self, msg):
        res = self.walk_srv()


def walking_again():

    rospy.init_node("walking_node")
    walking_again = Walking()

    rospy.spin()

if __name__ == "__main__":
    
    try:
        walking_again()
    except KeyboardInterrupt:
        print("Shutting down")
