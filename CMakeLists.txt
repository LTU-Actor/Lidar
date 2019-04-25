cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_route_obstacle)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  dynamic_reconfigure
  # obstacle loc
  pcl_ros
  message_generation
)

find_package(OpenCV REQUIRED)

##############
## Reconfig ##
##############

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Control.cfg
  cfg/RoadObstacle.cfg
  cfg/Region.cfg
)

add_message_files(
  FILES
  Region.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)



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
#  INCLUDE_DIRS include
#  LIBRARIES actor_obstacle
#  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}

  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  include

  # obstacle_loc
  ${PCL_INCLUDE_DIRS}
)

add_executable(obstacle_loc src/obstacle_loc.cpp)
add_dependencies(obstacle_loc ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_loc ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(region src/region.cpp)
add_dependencies(region ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(region ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
