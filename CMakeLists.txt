cmake_minimum_required(VERSION 2.8.3)
project(icra19_suture_rtt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cautery_description
  cautery_rtt
  cmake_modules
  endo360_description
  endo360_rtt
  geometry_msgs
  iiwa_description
  iiwa_hw
  iiwa_msgs
  kdl_conversions
  kdl_parser
  pcl_conversions
  roscpp
  rospy
  rtt_ros
  rtt_ros_msgs
  rtt_rosdeployment
  rtt_rosnode
  rtt_rosparam
  sensor_msgs
  star_rtt
  std_msgs
  tf_conversions
  urdf
  uta_description
  uta_rtt
)

find_package( Eigen3 REQUIRED )
find_package( PCL REQUIRED )
find_package( orocos_kdl REQUIRED )
find_package( OROCOS-RTT REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES icra19_suture
  CATKIN_DEPENDS cautery_description cautery_rtt cmake_modules endo360_description endo360_rtt geometry_msgs iiwa_descritption iiwa_hw iiwa_msgs kdl_conversions kdl_parser pcl_conversions roscpp rospy rtt_ros rtt_ros_msgs rtt_rosdeployment rtt_rosnode rtt_rosparam sensor_msgs std_msgs tf_conversions urdf uta_description uta_rtt star_rtt
  DEPENDS system_lib orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

orocos_component( suture_rtt src/suture_rtt.cpp )
add_dependencies( suture_rtt
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS} )
target_link_libraries(suture_rtt
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_conversions_LIBRARIES}
)
# add this one later  stitch )



