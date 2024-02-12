#! /usr/bin/env python3

import rospy
from std_msgs.msg import Empty as EmptyMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime

import cv2

class PictureNode:

    def __init__(self):
        self.cv_image = None
        self.video_writer = None
        self.start_time = None
        self.bridge = CvBridge()

        self.gpt_sub = rospy.Subscriber('/gpt_pictrue', EmptyMsg, self.gpt_callback)

    def gpt_callback(self, msg):

        self.take_picture()

    def take_picture(self):
        msg = rospy.wait_for_message("/camera_face/color/image_raw", Image, timeout=None)
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Save the image to file with current time appended to filename
        current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        filename = '/home/kimsooyoung/Pictures/received_image_{}.jpg'.format(current_time)
        cv2.imwrite(filename, cv_image)
        rospy.loginfo("Image saved as {}".format(filename))

def take_picture():

    rospy.init_node('picture_node', anonymous=True)
    picture_node = PictureNode()

    rospy.spin()

if __name__ == '__main__':

    try:
        take_picture()
    except KeyboardInterrupt:
        print("Shutting down")
