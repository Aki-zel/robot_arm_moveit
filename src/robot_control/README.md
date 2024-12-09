# Robot Control

## Overview

`robot_control`机械臂运动控制以及视觉检测代码。

## Architecture

### 1. robot_moveit_control
    机器臂控制中心，提供机械臂规划、执行、坐标转换等功能。

    # Components
    MoveitServer：Moveit服务端，提供机械臂规划、执行、坐标转换等功能。
    mainwindow：QT图形界面，可视化显示传感器图像以及机械臂状态。
    ArmControlCenter：机器臂的任务中心，利用视觉与控制接口实现机器臂开柜子、抓取目标、寻找柜子位置等任务。

    # Usage
    启动相机和机械臂控制节点：roslaunch robot_moveit_control robot_start.launch
    即可启动机械臂控制、打开相机并传入手眼标定参数

### 2. robot_camera_control
    相机控制中心，提供相机图像、相机参数以及相机坐标转换等功能。

    # Components
    

### 3. robot_msgs

### 4. utils

### 5. vgn

