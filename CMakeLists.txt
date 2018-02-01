cmake_minimum_required(VERSION 2.8.3)
project(rosbag_decoder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  kvaser
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosbag_decoder
#  CATKIN_DEPENDS cv_bridge image_transport kvaser pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

#find_package(Eigen REQUIRED)

find_package( OpenCV REQUIRED )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(PROJECT_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}


    )
set(SOURCE_FILE
    src/converter.cpp

    )
add_executable(${PROJECT_NAME}_node src/main.cpp ${SOURCE_FILE})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_LIBS}
 )

