cmake_minimum_required(VERSION 3.0.2)
project(people_tracking)

add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  darknet_ros_msgs
  track_pkg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES people_tracking
  CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        track_pkg
  LIBRARIES ${PROJECT_NAME}
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

add_executable(people_track_node
               src/people_tracking_node.cpp
               src/people_tracking_core.cpp
              )
target_link_libraries(people_track_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

