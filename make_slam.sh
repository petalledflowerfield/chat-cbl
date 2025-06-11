#!/bin/bash

exec 2>&1

export TURTLEBOT3_MODEL=burger

read -p "Bot brought up? (y/N)" yn
case $yn in
  [Yy]* ) ;;
  * ) read -p "IP " bot_ip
          ssh ubuntu@$bot_ip << EOF
export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL
ros2 launch turtlebot3_bringup robot.launch.py &
exit
EOF
          ;;
esac
          
ros2 launch turtlebot3_cartographer cartographer.launch.py > /dev/null &

read -p "Run teleop in a separate terminal and press enter when you're done mapping"

ros2 run nav2_map_server map_saver_cli -f $HOME/chat_turtlebot3_ws/maps
