cmake_minimum_required(VERSION 3.0.2)
project(imx219-83-ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imx219-83-ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/imx219-83-ros.cpp
# )

## Add cmake target dependencies of the library
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(usbcam_publisher src/usbcam_publisher.cpp)
add_executable(csicam_publisher src/csicam_publisher.cpp)
add_executable(cam_viewer src/cam_viewer.cpp)

target_link_libraries(usbcam_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(csicam_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cam_viewer ${catkin_LIBRARIES} ${OpenCV_LIBS})
