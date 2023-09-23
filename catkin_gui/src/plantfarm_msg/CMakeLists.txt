cmake_minimum_required(VERSION 3.0.2)
project(plantfarm_msg)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)

add_message_files(
  FILES
  YoloResult.msg
  YoloResultList.msg
  YoloKPT.msg
)

add_action_files(
  DIRECTORY action   # 액션 메시지 파일이 있는 디렉토리를 지정
  FILES
  dsr_moveline.action
  dsr_movejoint.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf message_runtime std_msgs actionlib_msgs
)

