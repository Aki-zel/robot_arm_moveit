# RWM_MOVEIT

## Overview

`RWM_MOVEIT`提供了规划、执行、和监控机械臂运动的功能，支持多种复杂的运动规划需求。本文档将介绍 `RWM_MOVEIT` 的功能、配置以及使用方法。

## Features

- **运动规划**：使用OMPL运动规划器。
- **路径执行**：通过机器人控制器执行规划的路径。
- **感知与环境**：整合传感器数据，支持动态环境更新。
- **交互界面**：提供QT可视化图形界面与用户交互。

## Architecture

`RWM_MOVEIT` 由以下几个主要组件组成：

### 1. robot_control
    机械臂控制中心
- **robot_moveit_control**：机械臂运动控制中心。
- **robot_camera_control**：视觉控制中心。
- **robot_helpers**：
- **robot_msgs**：目标检测以及任务服务消息。
- **utils**：辅助处理工具。
- **vgn**：机器人抓取姿态估计。

### 2. rm_robot
    睿尔曼机械臂ROS功能包

### 3. realsense-ros
    realsense相机ROS功能包

## Usage

### Starting robot_control

1. 启动相机和机械臂控制节点：roslaunch robot_moveit_control robot_start.launch
2. 选择相机检测服务：rosrun robot_camera_control object_detect.py(目标检测)
                  rosrun robot_camera_control color_detect.py(颜色阈值检测)
                  rosrun robot_camera_control template_detect.py(模版匹配)
3. 开启任务节点：rosrun robot_moveit_control controlcenter
4. 选择任务服务：rosservice call /call_task "taskId:1"
            taskId: 1(开门),2(开柜子)，3(抓取目标)，4(寻找柜子位置)
