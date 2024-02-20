#! /usr/bin/env python3

import os
import rospy

# from emergency_stop import e_stop
# from img_functions import take_picture, shoot_video
# from waypoint_control import target_waypoint
# from pitch_control import pitch_control
import openai 
import re
import argparse
import math
import numpy as np
import json
import time

parser = argparse.ArgumentParser()
parser.add_argument("--prompt", type=str, default="/home/kimsooyoung/unitree_ws/src/a1_chatgpt_demo/src/unitree_sys.txt")
parser.add_argument("--sysprompt", type=str, default="/home/kimsooyoung/unitree_ws/src/a1_chatgpt_demo/src/unitree_basic.txt")
args = parser.parse_args()

with open('/home/kimsooyoung/unitree_ws/src/a1_chatgpt_demo/src/config.json', 'r',encoding='UTF8') as f:
    config = json.load(f)

print("Initializing ChatGPT...")
openai.api_key = config["OPENAI_API_KEY"]

with open(args.sysprompt, "r",encoding='UTF8') as f:
    sysprompt = f.read()

chat_history = [
    {
        "role": "system",
        "content": sysprompt
    },
    {
        "role": "user",
        "content": "현재 기준에서 앞으로 1m 이동해"
    },
    {
        "role": "assistant",
        "content": """```python
function_handler.target_waypoint(1, 0, 0)
```

이 코드는 로봇이 현재 위치에서 x축 방향으로 3미터 앞으로 이동하도록 합니다. target_waypoint 함수를 사용할 때, 이동 완료까지 충분한 시간을 기다려야 한다는 점을 기억해야 합니다. 1미터 이동에 4초가 필요하므로, 1미터 이동에는 대략 4초의 대기 시간을 고려해야 할 수 있습니다. """
    }
]


def ask(prompt):
    chat_history.append(
        {
            "role": "user",
            "content": prompt,
        }
    )
    completion = openai.ChatCompletion.create(
        # model="gpt-3.5-turbo",
        model="gpt-4",
        messages=chat_history,
        temperature=0.5
    )
    chat_history.append(
        {
            "role": "assistant",
            "content": completion.choices[0].message.content,
        }
    )
    return chat_history[-1]["content"]


print(f"Done.")

code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)


def extract_python_code(content):
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)

        if full_code.startswith("python"):
            full_code = full_code[7:]

        return full_code
    else:
        return None



from function_handler import GPTFunctionHandler

class colors:  # You may need to change color settings
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"

rospy.init_node("chatgpt_quadruped_node")
function_handler = GPTFunctionHandler()


with open(args.prompt, "r", encoding='UTF8') as f:
    prompt = f.read()

ask(prompt)
print("Welcome to the unitree simulator! I am ready to help you")

function_handler.e_stop() 

while True:
    question = input(colors.YELLOW + "ChatGPT Quadruped> " + colors.ENDC)

    if question == "!quit" or question == "!exit":
        break

    if question == "!clear":
        os.system("cls")
        continue

    response = ask(question)
    print(f"\n{response}\n")

    code = extract_python_code(response)
    if code is not None:
        print("Please wait while I run the code...")
        exec(extract_python_code(response))
        print("Done!\n")