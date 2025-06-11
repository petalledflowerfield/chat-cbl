#!/bin/bash

git submodule update --init

export TURTLEBOT3_MODEL=burger

read -p "IP " bot_ip

ssh ubuntu@$bot_ip << EOF
export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL
ros2 launch turtlebot3_bringup robot.launch.py &
ros2 run usb_cam usb_cam_node_exe &
exit
EOF

ws_path=~/chat_turtlebot3_ws

mkdir -p $ws_path/src

cp -r crowd_nav2 fake_node phyvir signaling smart_explorer trash_map yolo_detector turtlebot3_simulations $ws_path/src/

cd $ws_path

colcon build
source install/setup.bash

read -p "Press enter to launch virtual environment"

ros2 launch turtlebot3_gazebo first_world.launch.py

read -p "Place the robot in the same location as in the virtual environment"
read -p "Run teleop in a separate window and move the robot around to check the scale of the virtual environment with the physical environment (if adjustments needed, exit with Ctrl-C)"
read -p "Press enter to run necessary nodes"

ros2 run phyvir phyvir_passthru &
ros2 run smart_explorer smart_explorer &
ros2 run yolo_detector yolo_detector &
ros2 run signaling signaling &
