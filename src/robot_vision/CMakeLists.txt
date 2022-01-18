cmake_minimum_required(VERSION 3.0.2)
project(robot_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  roslaunch
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  tm_msgs
  geometry_msgs
  tf
)

roslaunch_add_file_check(launch)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  YoloBoundingBox.msg
  ArUcoList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PoseTrans.srv
  # Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES robot_vision
 CATKIN_DEPENDS message_runtime cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}  src/RotationTools.cpp)

add_executable(imgProccess src/imgProccess.cpp) 
target_link_libraries(imgProccess ${catkin_LIBRARIES})

add_executable(aruco_detect src/aruco_detect.cpp) 
target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(tf_boardcast src/tf_boardcast.cpp) 
target_link_libraries(tf_boardcast ${catkin_LIBRARIES})
add_dependencies(tf_boardcast ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(aruco_organize src/aruco_organize.cpp) 
target_link_libraries(aruco_organize ${catkin_LIBRARIES})
#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pub_upper_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/yolo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
