#!/bin/bash

# Commands and titles
commands=(
  "ros2 launch mtc_tutorial mtc_demo.launch.py"
  "ros2 launch mtc_tutorial pick_place_demo.launch.py"
  "ros2 run mtc_tutorial task_planner_service.py"
  "ros2 run mtc_tutorial udp_joint_pos_sender"
  "ros2 run mtc_tutorial mtc_action_client"
)

titles=(
  "mtc_demo.launch.py"
  "pick_place_demo.launch.py"
  "task_planner_service.py"
  "udp_joint_pos_sender"
  "mtc_action_client"
)

# Launch each command in a new terminal tab with 1s delay
for i in "${!commands[@]}"; do
  gnome-terminal --tab --title="${titles[$i]}" -- bash -c "${commands[$i]}; exec bash"
  sleep 1
done

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      