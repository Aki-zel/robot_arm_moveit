#!/bin/bash

# 启动 ROS master 节点
source /opt/ros/noetic/setup.bash    # 替换为你的 ROS 版本，例如 melodic 或 noetic
source ~/rwm_moveit/devel/setup.bash  # 替换为你的工作空间路径
source ~/camport_ros/devel/setup.bash
roscore &  # 后台启动 roscore
sleep 2    # 等待 roscore 完全启动
ROSCORE_PID=$!  # 获取 roscore 的进程 ID

# 启动 ROS launch 文件
roslaunch robot_moveit_control chessGameStart.launch &  # 替换为你的 launch 文件路径
LAUNCH_PID=$!  # 获取 roslaunch 的进程 ID
# 等待 ROS launch 启动完成
sleep 10  # 等待5秒，确保 launch 文件中的节点已启动

# 启动五子棋例程
rosrun robot_moveit_control example05p

# 捕捉 SIGINT (Ctrl+C) 和 SIGTERM 信号，确保在关闭时终止所有进程
trap "echo 'Terminating...'; kill $LAUNCH_PID; kill $ROSCORE_PID; exit" SIGINT SIGTERM
wait $LAUNCH_PID # 等待 roslaunch 完成
wait $ROSCORE_PID # 等待 roscore 进程结束
echo "ROS nodes and roscore terminated successfully."