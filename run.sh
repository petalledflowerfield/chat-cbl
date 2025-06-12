#!/bin/bash

exec 2>&1

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
cp -r model /tmp

cd $ws_path

colcon build
source install/setup.bash

read -p "Press enter to launch virtual environment"

ros2 launch turtlebot3_gazebo first_world.launch.py > /dev/null &

echo "ROS2 topics:"
ros2 topic list -v

read -p "Place the robot in the same location as in the virtual environment"
read -p "Run teleop in a separate window and move the robot around to check the scale of the virtual environment with the physical environment (if adjustments needed, exit with Ctrl-C)"

read -p "Do you have a slam map yet? (y/N)" yn
case $yn in
  [Yy]* ) ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$(pwd)/maps/map.yaml > /dev/null & ;;
  * ) echo "Go run make_slam.sh first"
      exit;;
esac

read -p "Calibrate the navigation module, then press enter to run necessary nodes"

ros2 run phyvir phyvir_passthru &

read -p "Check if phyvir is working as expected (next is yolo_detector)"

ros2 run yolo_detector yolo_detector &

read -p "Check if yolo_detector is working as expected (next is signaling)"

ros2 run signaling signaling &

read -p "Check if signaling is working as expected (next is smart_explorer)"

ros2 run smart_explorer smart_explorer
