# Unitree A1 ChatGPT Demo

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