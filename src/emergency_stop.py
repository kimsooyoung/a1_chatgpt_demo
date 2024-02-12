#! /usr/bin/env python3

import time
import rospy
import numpy as np
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse


class EStop(object):

    def __init__(self):

        self.stand_srv = rospy.ServiceProxy("/standing", Empty)
        self.gpt_sub = rospy.Subscriber('/gpt_standing', EmptyMsg, self.callback)

    def callback(self, msg):
        res = self.stand_srv()

        rospy.loginfo("Emergency Stop!")


def e_stop():

    rospy.init_node("e_stop_node")
    e_stop = EStop()

    rospy.spin()

if __name__ == "__main__":
    
    try:
        e_stop()
    except KeyboardInterrupt:
        print("Shutting down")
