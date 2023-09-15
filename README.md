# 2023capstone
Capstone design project

It's a modification of the original code. I did UI work on the original code.

Original code repository : 
[Plantfarm project by Song](https://gitlab.com/Alsrbile/2023-capstone/-/tree/v2.0.0?ref_type=heads)

## Enviroments
Doosan-Robotics collaborative robot m1013 <br>
Intel-RealSense 435i<br>
SCHUNK EGP gripper<br>

## Build
This program is implemented at Ubuntu 20.04 - ROS-Noetic

#### 0) ROS-Noetic
[http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)<br>

Our project needs the packages written below.<br>


#### 1) Doosan-Robotics noetic-devel 
[https://github.com/doosan-robotics/doosan-robot#overview](https://github.com/doosan-robotics/doosan-robot#overview)<br>

#### 2) Intel-Realsense
[https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)<br>

#### 3) PCL
[https://github.com/PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)<br>

Cuda environment is **cuda 11.6+cudnn 8.2.0** <br>
Pytorch version is **1.13.1+cu116**

#### 4) Yolo v7

[https://gitlab.com/Alsrbile/2023-capstone/-/tree/main/catkin_test/src/yolov7-u7/src/seg?ref_type=heads](https://gitlab.com/Alsrbile/2023-capstone/-/tree/main/catkin_test/src/yolov7-u7?ref_type=heads)<br>

You should download this package.<br>
You only need this package from the original repository.<br>
Put **"predict_ui.py"** in the etc folder to this path **"/catkin_test/src/yolov7-u7/src/seg/segment"**.

# How to use?

1. JUST RUN!!!!
> roslaunch roslaunch plantfarm_ui plantfarm_ui_go.launch 
2. PLUG AND PLAY use UI.
