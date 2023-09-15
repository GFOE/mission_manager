cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  tf2
  message_generation
  dynamic_reconfigure
  path_follower
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/mission_manager.cfg
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Behavior.action
#   Action2.action
# )
 
 ## Generate added messages and services with any dependencies listed here
 # generate_messages(
 #  DEPENDENCIES
 #  std_msgs  # Or other packages containing msgs
 #  actionlib_msgs
 #)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES missionmanager
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

