cmake_minimum_required(VERSION 2.8.3)
project(srrc_simple_obj_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
#  image_geometry
#  image_proc
#  image_transport
#  message_filters
  roscpp
#  rospy
  sensor_msgs
  #stereo_image_proc
#  stereo_msgs
  std_msgs
#  tf
)

# OpenCV
find_package(OpenCV)
IF (OPENCV_FOUND)
  MESSAGE("-- Found OpenCV version ${OPENCV_VERSION}: ${OPENCV_INCLUDE_DIRS}")
ENDIF (OPENCV_FOUND)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES srrc_img2dir
  CATKIN_DEPENDS
    cv_bridge
#    image_geometry
#    image_proc
#    image_transport
    roscpp
#    sensor_msgs
    #stereo_image_proc
#    stereo_msgs
#    std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(test_simple_detector_node
  test_simple_detector_node.cpp
)
## Specify libraries to link a library or executable target against
target_link_libraries(test_simple_detector_node
   ${catkin_LIBRARIES}
 )

