cmake_minimum_required(VERSION 3.0.2)
project(tm_helper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tm_msgs
  tf
)
# roslaunch_add_file_check(launch)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

#lib
add_library(${PROJECT_NAME}  src/tmrobot.cpp)


include_directories(  
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(jog 
src/jog.cpp
src/tmrobot.cpp
) 
target_link_libraries(jog Qt5::Widgets ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES})

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