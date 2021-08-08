# Overview

Repository containing the 'mission_manager_node.py' ROS node which uses SMACH for heirarchical state machine implementation.

# Command Strings

A "command" is sent as a ROS message String and includes a command and an optional argument, separated by whitespace, i.e, `command_str = "cmd cmd_args"`. Below is a description of the various command strings, their syntax and the resulting behavior.

## cmd = "replace_task"

## cmd = "append_task"

## cmd = "prepend_task"

## cmd = "clear_tasks"

## cmd = next_task, prev_task, goto_task, goto_line, start_line or restart_mission

## cmd = "override"


# Task Definition Strings and Task Types

Tasks are sent via the Command string interface (and possibly other ways).
The Task String containts "task_type task_args" deliminted by whitespace, i.e., `task_str = "task_type task_args"`.  The task_types and their task_args are described below.  

Tasks are stored in a list (queue) as the `tasks` attribute of the 


## task_type = "mission_plan"

task_arg is complete mission in json format.  Not sure the syntax of a mission.

## task_type = "goto"

task_arg = "latitude longitude", where both are in decimal degrees.

## hover

task_arg = "latitude longitude", where both are in decimal degrees.

 
# State Strings

The "State" (of what, I'm not sure) is reported in the published Heartbeat message as one of the key/value pairs.

Possible states strings and their meanings are...

## Idle

## Hover

## FollowPath

## SurveyArea

# Status Heartbeat Messages

The message includes a number of key/value pairs.  All keys and values are exclusivly strings.  Here are the keys we are aware of...

## state

The state string (see above)

## task_count

Length of the tasks list in the mission_manager object - as a string of course.

# Pending Command Strings

The `pending_command` attribute of the MissionManagerCore object is a string.  Enumeration and explanation of the strings are aas follows:

## do_override
