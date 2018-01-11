cmake_minimum_required(VERSION 2.8.3)
project(kumonoito)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(drake CONFIG REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
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
  LIBRARIES kumonoito
  CATKIN_DEPENDS roscpp std_msgs genmsg
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

# Libraries

add_library(jaco_common src/jaco_common.cc
        src/controlled_jaco_simulation.cc
        src/ros_subscriber_system.cc src/ros_publisher_system.cc src/robot_table_planner.cc)
target_link_libraries(jaco_common drake::drake)

add_library(ros_joint_state_publisher src/ros_joint_state_publisher.cc)
target_link_libraries(ros_joint_state_publisher
        drake::drake
        ${catkin_LIBRARIES})

add_library(ros_tf_publisher src/ros_tf_publisher.cc)
target_link_libraries(ros_tf_publisher
        drake::drake
        ${catkin_LIBRARIES})

add_library(ros_publisher_system src/ros_publisher_system.cc)
target_link_libraries(ros_publisher_system
        drake::drake
        ${catkin_LIBRARIES})


add_library(ros_subscriber_system src/ros_subscriber_system.cc)
target_link_libraries(ros_subscriber_system
        drake::drake
        ${catkin_LIBRARIES})

install(TARGETS jaco_common ros_joint_state_publisher ros_tf_publisher ros_publisher_system ros_subscriber_system
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Binaries

add_executable(jaco_passive_simulation_demo src/jaco_passive_simulation_demo.cc)
target_link_libraries(jaco_passive_simulation_demo
        jaco_common
        drake::drake
#        gflags
        ros_joint_state_publisher
        ${catkin_LIBRARIES})

add_executable(controlled_jaco_simulation_demo src/controlled_jaco_simulation.cc)
target_link_libraries(controlled_jaco_simulation_demo
        jaco_common
        drake::drake
        ros_joint_state_publisher
        ros_tf_publisher
        ros_subscriber_system)

install(TARGETS jaco_passive_simulation_demo controlled_jaco_simulation_demo
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#
## WIP : Planner process that publishes JointTrajectory
#add_executable(robot_table_planner src/robot_table_planner.cc)
#target_link_libraries(robot_table_planner
#        jaco_common
#        drake::drake
#        ros_joint_state_publisher
#        ros_publisher_system)

#############
## Install ##
#############

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
