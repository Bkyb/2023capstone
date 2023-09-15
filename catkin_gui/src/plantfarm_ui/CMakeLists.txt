cmake_minimum_required(VERSION 3.10)
project(plantfarm_ui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  pcl_ros
  dsr_msgs
  plantfarm_msg
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include additional ROS packages if needed
# find_package(포함할_ROS_패키지 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED)
find_package(OpenCV 4.5.0 REQUIRED)
find_package(realsense2 REQUIRED)

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
  yaml-cpp
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  plantfarm_msg
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES plantfarm_ui
    CATKIN_DEPENDS roscpp rospy tf message_runtime std_msgs plantfarm_msg
    DEPENDS
        Qt5Widgets
        Qt5PrintSupport
        system_lib	
)



# List of nodes
# set(NODES
#  "node1.cpp"
#  "node2.cpp"
#  # Add more nodes as needed
#)

file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)

set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

# find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

# Create a list of executable targets
set(EXECUTABLES_LIST "")

message("Building ${PROJECT_NAME} nodes: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES} ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND EXECUTABLES_LIST ${NODE_NAME})
endforeach(NODE_FILE)

# Install all executable targets
install(TARGETS ${EXECUTABLES_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

