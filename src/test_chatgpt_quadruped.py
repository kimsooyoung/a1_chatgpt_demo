#! /usr/bin/env python3

import os
import rospy

# from emergency_stop import e_stop
# from img_functions import take_picture, shoot_video
# from waypoint_control import target_waypoint
# from pitch_control import pitch_control

from function_handler import GPTFunctionHandler

class colors:  # You may need to change color settings
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"

rospy.init_node("chatgpt_quadruped_node")
function_handler = GPTFunctionHandler()

while True:
    question = input(colors.YELLOW + "ChatGPT Quadruped> " + colors.ENDC)

    if question == "!quit" or question == "!exit":
        break

    if question == "!clear":
        os.system("cls")
        continue

    print(question)
    exec(question)