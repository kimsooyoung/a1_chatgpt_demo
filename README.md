# Unitree A1 ChatGPT Demo
> Simple Demo for Robotics and LLM Integration. Unitree A1 quadruped robot and ChatGPT 4.0 utilized

## Project Structure

1. Write prompt including how to use robot API and the information about the scene.
2. Ask abstract or specific questions to ChatGPT
3. GPT will make codes for your questions with received prompts.
4. Simulation test and improve prompt and give more clues for more precise planning.
5. [Ongoing] Real Robot Demo 

![image](https://github.com/kimsooyoung/a1_chatgpt_demo/assets/12381733/3718b4ba-4389-4dbd-bcfb-9191b8ef6aef)


## Demo Video

https://github.com/kimsooyoung/a1_chatgpt_demo/assets/12381733/68a2cc1a-18e8-465c-880b-897f3af7ba8a

### Usage

```
# Terminal 1
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=empty_world
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=no_roof_small_warehouse

# Terminal 2
# let the robot stretch legs
# 다리를 펴는 용도로만 쓰인다. 한 번 쓰고 실행 취소
rosrun unitree_controller unitree_servo 
# place the robot back to origin
# 위치를 원점으로 움직이는 용도로 쓰인다. 한 번 쓰고 실행 취소
rosrun unitree_controller unitree_move_kinetic 

# Terminal 3
roslaunch a1_chatgpt_demo a1_demo.launch

# Terminal 4
rosrun a1_chatgpt_demo chatgpt_quadruped.py
```

⚠️ You will need `config.json` file for ChatGPT API key
