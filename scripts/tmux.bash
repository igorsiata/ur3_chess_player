#!/usr/bin/env bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WS_DIR="$SCRIPT_DIR/../ros2_ws"
cd "$WS_DIR"
source /opt/ros/humble/setup.bash
echo "Building workspace..."
colcon build 

SESSION="ros2_session"
tmux new-session -d -s $SESSION

# -----------------------------
# Docker ur3
# -----------------------------
tmux new-window -t $SESSION:6 -n "docker_ur3"
tmux send-keys -t $SESSION:6 "docker run --rm -it e76" C-m

sleep 1
# -----------------------------
# UR DRIVER
# -----------------------------
tmux new-window -t $SESSION:5 -n "ur_driver"
tmux send-keys -t $SESSION:5 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:5 "ros2 launch ur_robot_driver ur3.launch.py robot_ip:=192.168.0.10 launch_rviz:=false" C-m

# -----------------------------
# GRIPPER DRIVER
# -----------------------------
tmux new-window -t $SESSION:4 -n "gripper_driver"
tmux send-keys -t $SESSION:4 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:4 "ros2 run computer_opponent gripper_driver" C-m

sleep 2
# -----------------------------
# move_maker
# -----------------------------
tmux new-window -t $SESSION:3 -n "move_maker"
tmux send-keys -t $SESSION:3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION:3 "ros2 launch ur3_tcp ur_moveit2.launch.py" C-m

# -----------------------------
# WS_2
# -----------------------------
tmux new-window -t $SESSION:2 -n "ws_2"
tmux send-keys -t $SESSION:2 "source install/setup.bash" C-m

# -----------------------------
# WS_1
# -----------------------------
tmux rename-window -t $SESSION:1 "ws_1"
tmux send-keys -t $SESSION:1 "source install/setup.bash" C-m

# Na koniec przełącz do sesji
tmux attach -t $SESSION

# tmux kill-session -t ros2_session
