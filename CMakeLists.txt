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
  path_follower
  project11_nav_msgs
  
)

catkin_python_setup()

add_service_files(
  FILES
  TaskInformationIdList.srv
  TaskInformationList.srv
  TaskManagerCmd.srv
)

generate_messages(
        DEPENDENCIES
        project11_nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES missionmanager
  CATKIN_DEPENDS rospy message_runtime std_msgs geometry_msgs nav_msgs project11_nav_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

