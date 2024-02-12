#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64
import datetime

import cv2

class VideoNode:

    def __init__(self):
        self.cv_image = None
        self.video_writer = None
        self.start_time = None
        self.bridge = CvBridge()
        self.recording_duration = 0.0

        self.video_flag = False

        self.image_sub = rospy.Subscriber('/camera_face/color/image_raw', Image, self.image_callback)
        self.gpt_sub = rospy.Subscriber('/gpt_video', Float64, self.gpt_callback)

    def gpt_callback(self, msg):
        rospy.loginfo("GPT CB")
        self.recording_duration = msg.data
        self.video_flag = True

    def image_callback(self, msg):

        if self.video_flag is False:
            return

        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            return

        # Initialize video writer if not initialized
        if self.video_writer is None and self.cv_image is not None:
            self.start_time = rospy.get_time()
            self.current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            filename = '/home/kimsooyoung/Pictures/received_video_{}.avi'.format(self.current_time)
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (self.cv_image.shape[1], self.cv_image.shape[0]))

        if self.cv_image is not None:
            # Write frame to video
            self.video_writer.write(self.cv_image)
        
        time_dt = rospy.get_time() - self.start_time
        print("time_dt: ", time_dt)

        # Check if recording duration exceeded
        if time_dt > self.recording_duration:
            rospy.loginfo("Recording stopped")
            self.video_writer.release()
            self.video_writer = None
            self.video_flag = False

def shoot_video():

    rospy.init_node('video_node', anonymous=True)
    video_node = VideoNode()

    rospy.spin()

if __name__ == '__main__':

    try:
        shoot_video()
    except KeyboardInterrupt:
        print("Shutting down")
