#!/usr/bin/env python3

import rospy

from std_msgs.msg import String
from project11_msgs.msg import Heartbeat
from project11_msgs.msg import KeyValue
from project11_msgs.msg import BehaviorInformation
from project11_nav_msgs.msg import TaskInformation
from geometry_msgs.msg import PoseStamped
import project11

import actionlib
import project11_navigation.msg
from project11_msgs.msg import BehaviorInformation
import json
import yaml


def parseLatLong(args):
    """ Splits a string into latitude and longitude.

    Splits a string in two and crates a dictionary with 
    latitude and longitude keys and float values.

    Args:
        args: 
        A str of two float numbers separated by whitespace.  
    
    Returns:
        A dict with keys 'latitude' and 'longitude' and float values.
    """
    latlon = args.split()
    if len(latlon) == 2:
        try:
            lat = float(latlon[0])
            lon = float(latlon[1])
            return {'latitude':lat, 'longitude':lon}
        except ValueError:
            rospy.logerr("mission_manager: Cannot convert the command "
                            "arguments <%s> into two floats!"%args)
            return None
    else:
        rospy.logerr("mission_manager: Cannot split the command "
                        "arguments <%s> into exactly two elements!"%args)

    return None


class MissionManager(object):
    def __init__(self):

        """ 
        List of tasks to do, or already done.
        Keeping all the tasks allows us to run them in a loop
        Elements of the list are dictionary objects - see addTask()
        """
        self.tasks = []
        # A task that may be added, such as hover,
        # to temporarily interrupt current task.
        self.override_task = None
        self.done_task = TaskInformation()
        self.done_task.type = "hover"
        self.done_task.id = "done_hover"
        self.done_task.priority = 100

        # A dictionary of behaviors, keyed on task.id.
        self.behaviors_library = {}
        self.active_behaviors = []

        self.command_subscriber = rospy.Subscriber('project11/mission_manager/command', 
                                                   String, self.commandCallback,
                                                    queue_size = 1)

        self.status_publisher = rospy.Publisher('project11/status/mission_manager', 
                                                Heartbeat, queue_size = 10)

        self.earth = project11.nav.EarthTransforms()

        self.navigator_client = actionlib.SimpleActionClient('navigator/run_tasks', 
                                                             project11_navigation.msg.RunTasksAction)
        # A place to hold the running behavior information publishers.
        self.behavior_library = {}
        self.behavior_info_publishers = {}
        self.behavior_feedback_subscribers = {}

        self.updateNavigator()

    def updateNavigator(self):
        goal = project11_navigation.msg.RunTasksGoal()
        if(self.override_task is not None):
            goal.tasks.append(self.override_task)
        for t in self.tasks:

            # Populate the library with the behaviors for this task.
            self.behavior_library[t.id] = t.behaviors

            for bhv in t.behaviors:
                # Don't make new pub/sub's if they already exist.
                if bhv.id not in self.behavior_info_publishers.keys():
                    # Create behavior publisher and subscriber
                    self.behavior_info_publishers[bhv.id] = rospy.Publisher('project11/behaviors/' +
                                                                        bhv.type + 
                                                                        '/input',
                                                                        BehaviorInformation,
                                                                        queue_size=10,
                                                                        latch=True)
            
                    self.behavior_feedback_subscribers[bhv.id] = rospy.Subscriber('project11/behaviors/' +
                                                                        bhv.type +
                                                                        '/feedback',
                                                                        BehaviorInformation,
                                                                        queue_size=1)
                print("publishing behavior: %s" % bhv)
                self.behavior_info_publishers[bhv.id].publish(bhv)

            # Send the behavior info here for each? or Wait until feedback from the navigator?
            

            goal.tasks.append(t)

        goal.tasks.append(self.done_task)
        if self.navigator_client.wait_for_server(rospy.Duration(2.0)):
            rospy.loginfo("mission_manager: sending goal:")
            rospy.loginfo(goal)
            self.navigator_client.send_goal(goal,
                                            active_cb = self.navigatorActiveCallback,
                                            feedback_cb = self.navigatorFeedbackCallback,
                                            done_cb = self.navigatorDoneCallback)
        else:
            rospy.logwarn('Timeout waiting for navigator action server')

    def navigatorActiveCallback(self):
        rospy.loginfo("navigator active")
        pass

    def listTasks(self, feedback, hb):
        if feedback is None:
            hb.values.append(KeyValue("tasks","none"))
            return
        if len(feedback.tasks) == 0:
            hb.values.append(KeyValue("tasks","empty"))
        for task in feedback.tasks:
            kv = KeyValue()
            kv.key = task.id
            kv.value = "type: "+task.type
            if task.done:
                kv.value += " (done)"
            if len(task.status):
                kv.value += " status: " + str(task.status)
            hb.values.append(kv)

    def navigatorFeedbackCallback(self, feedback):

        # This block needs a comment to exlain what's going on here.
        needUpdate = False
        if feedback is not None:
            for updated_task in feedback.tasks:
                if self.override_task is not None and self.override_task.id == updated_task.id:
                    if updated_task.done:
                        self.override_task = None
                        needUpdate = True
                for task in self.tasks:
                    if task.id == updated_task.id:
                        task.done = updated_task.done

        print(feedback.current_nav_task)

        # Activate behaviors for each task:
        if feedback is not None:
            if feedback.current_nav_task in self.behavior_library.keys():
                # Loop through each behavior and activate/update them.
                for bhv in self.behavior_library[feedback.current_nav_task]:
                    print("publishing behavior %s" % bhv.id)
                    print(bhv)
                    self.behavior_info_publishers[bhv.id].publish(bhv)
        else:
            rospy.logwarn('Did not activate behaviors. Timeout waiting for navigator feedback')


        hb = Heartbeat()
        hb.values.append(KeyValue("Navigator","active"))
        if feedback is not None:
            hb.values.append(KeyValue("Current Nav Task", feedback.current_nav_task))
        self.listTasks(feedback, hb)
        self.status_publisher.publish(hb)

        if needUpdate:
            self.updateNavigator()

    def navigatorDoneCallback(self, state, result):
        hb = Heartbeat()
        hb.values.append(KeyValue("Navigator","done"))
        self.listTasks(result, hb)
        self.status_publisher.publish(hb)

    
    def behaviorFeedbackCallback(self,feedback):
        rospy.loginfo(feedback)
        pass

    def commandCallback(self, msg):
        """ Receives ROS command String.

        Args:
          msg: 
            A std_msg/String message.
            Formatted string, delimited by whitespace, describing task_type 
            and task parameters.
        """
        
        parts = msg.data.split(None,1)
        cmd = parts[0]
        if len(parts) > 1:
            args = parts[1]
        else:
            args = None
                
        if cmd == 'replace_task':
            self.tasks = []
            self.addTask(args)
        elif cmd == 'append_task':
            self.addTask(args)
        elif cmd == 'prepend_task':
            self.addTask(args, True)
        elif cmd == 'clear_tasks':
            rospy.loginfo('mission_manager: clear navigation tasks.')
            self.tasks = []
        elif cmd in ('next_task','prev_task','goto_task',
                   'goto_line', 'start_line', 'restart_mission'):
            if cmd == 'next_task' and self.override_task is not None:
                self.override_task = None
            # TODO, manipulate task status in response to commands
            #self.pending_command  = msg.data
        elif cmd == 'cancel_override':
            self.override_task = None
        elif cmd == 'override':
            parts = args.split(None,1)
            if len(parts) == 2:
                task_type = parts[0]   
                if task_type == 'goto':
                    task = TaskInformation()
                    task.type = "goto"
                    task.id = "goto_override"
                    task.priority = -1
                    ll = parseLatLong(parts[1])
                    if ll is not None:
                        task.poses.append(self.earth.geoToPose(ll['latitude'], ll['longitude']))
                        self.override_task = task
                elif task_type == 'hover':
                    rospy.loginfo('mission_manager: hover.')
                    task = TaskInformation()
                    task.type = "hover"
                    task.id = "hover_override"
                    task.priority = -1
                    ll = parseLatLong(parts[1])
                    if ll is not None:
                        task.poses.append(self.earth.geoToPose(ll['latitude'], ll['longitude']))
                    self.override_task = task
                elif task_type == 'idle':
                    task = TaskInformation()
                    task.type = "idle"
                    task.id = "idle_override"
                    task.priority = -1
                    self.override_task = task
        else:
            rospy.logerr("mission_manager: No defined action for the "
                         "received command <%s> - ignoring!"%msg.data)

        self.updateNavigator()
        

    def addTask(self, args, prepend=False):
        """ Appends or prepends an element to the "tasks" list attribute.

        Called when "append_task" or "prepend_task" commands are received.

        Tasks are dictionaries with a variety of keys.  Each dictionary 
        includes a 'type' key.

        Args:
          args: 
            A str that is the task definition string (see README.md)
            The remainder of the string sent with the command.
            See README.md for task string syntax.
          prepend: A bool to prepend (true) or append (false).
        """
        parts = args.split(None,1)
        rospy.loginfo("mission_manager: Adding task with arguments: %s"%parts)
        if len(parts) == 2:
            task_type = parts[0]
            task_list = []
            if task_type == 'mission_plan':
                tasks_and_behaviors = self.parseMission(json.loads(parts[1]))
                task_list = tasks_and_behaviors['tasks']
                if len(tasks_and_behaviors['behaviors']) > 0:
                    rospy.logerr("mission_manager: top level behaviors not yet supported")
            # elif task_type == 'goto':
            #     task = parseLatLong(args)
            #     if task is not None:
            #         task['type'] = 'goto'
            #         task_list.append(task)
            # elif task_type == 'hover':
            #     task = parseLatLong(args)
            #     if task is not None:
            #         task['type'] = 'hover'
            #         task_list.append(task)
            else:
                rospy.logerr("mission_manager: No defined task of type <%s> "
                             "from task string <%s>"%(task_type, args))
                
            if len(task_list): 
                if prepend:
                    self.tasks = task_list + self.task
                else:
                    self.tasks += task_list
            else:
                rospy.logerr("mission_manager: The task string <%s> was "
                             "not successfully parsed. No task added!"%
                             args)
        else:
            rospy.logerr("mission_manager: Task string <%s> was not "
                         "split into exactly two parts.  No task added!")
            
        rospy.loginfo('mission_manager: tasks : %s'%str(self.tasks))

    def newTaskWithID(self, item, parent_id='', id='task'):
        task = TaskInformation()
        if 'label' in item:
            task.id = parent_id+item['label']
        else:
            task.id = parent_id+id
        if 'speed' in item:
            task.data = yaml.safe_dump({'speed': item['speed']*0.514444})  # knots to m/s
        if 'children' in item:
            for sub_item in item['children']:
                if sub_item['type'] == 'Behavior':
                    task.behaviors.append(self.parseBehavior(sub_item))

        return task


    def parseWaypoints(self, items, task):
        waypoints = []
        for i in items:
            if 'type' in i and i['type'] == 'Waypoint':
                waypoints.append(i)

        heading = 0.0
        for i in range(len(waypoints)):
            wp = waypoints[i]
            if i+1 < len(waypoints):
                next_wp = waypoints[i+1]
                heading = project11.nav.distanceBearingDegrees(wp['latitude'],
                                                                wp['longitude'],
                                                                next_wp['latitude'],
                                                                next_wp['longitude'])[1]
            task.poses.append(self.earth.geoToPose(wp['latitude'], wp['longitude'], heading))
        return task

    def parseTrackLine(self, item, parent_id, id='line'):
        task = self.newTaskWithID(item, parent_id, id)
        task.type = "survey_line"
        try:
            self.parseWaypoints(item['children'], task)
            for sub_item in item['children']:
                print("SUBITEM")
                print(sub_item)
                if sub_item['type'] == 'Behavior':
                    task.behaviors.append(self.parseBehavior(sub_item))
            #self.parseBehaviors(item,task)
        except KeyError:
            rospy.logwarn('"children" not found in ', item)
        return task
    
    def parseBehavior(self, item):
        '''Parse a single behavior mission item'''
        behavior = BehaviorInformation()
        try:
            behavior.id = item['id']
        except:
            behavior.id = item['behaviorType']
        behavior.type = item['behaviorType']
        behavior.enabled = item['enabled'].lower()=='true'
        behavior.data = yaml.safe_dump(item['data'])
        return behavior


    def parseBehaviors(self, item, task):
        '''Parse mission element behaviors into YAML for addition to the 
        TaskInformation.msg.
        
        Args:
            plan: A string in JSON format describing a mission, usually sent 
            from a mission planner to the mission manager. When this mission
            contains a 'behaviors' list, they are parsed into yaml format by
            this method.

        Returns:
            A yaml formatted string containing the behavior elements.
            
        '''

        if 'behaviors' not in item.keys():
            return
        
        # Strategy:
        # In the event that there is already task data, we don't want to over
        # write it. But you can't concatenate YAML, so we load what's already there 
        # as a python dictionary, concatenate the old with the new, convert back
        # to YAML, and then set the task.data to the combined. 
        datatmp = {}
        if task.data != '':
            datatmp = yaml.safe_load(task.data)

        behaviortmp = {"behaviors":item['behaviors']}
        print(behaviortmp)
        # This method of merging will have the effect of replacing any fields
        # that already exist in the data block with new ones specified in the 
        # new behavior data, when they have the same keys.  
        mergeddata = {**datatmp, **behaviortmp}
        task.data = yaml.dump(mergeddata)
        rospy.loginfo("parsed behavior fields in task %s,\"%s\":\n %s" %
                      (item['type'],item['label'],task.data))

            
        return 

    def parseMission(self, plan, parent_id='', ignore_list = []):
        """ Create a task dict from a json description.
        
        Called when a "mission_plan" command is received.

        TODO: The interface changed - need to check documentation.
        
        Args:
            plan: A str in json format describing a mission. 

        Returns:
            A dict describing the task dictionary for mission 
            described by the plan json.
        """
        ret = {'tasks':[], 'behaviors':[]}
        
        for item in plan:
            rospy.loginfo("mission_manager: Parsing new mission item <%s>"%item)

            if item['type'] == 'Waypoint' and not 'Waypoint' in ignore_list:
                task = self.newTaskWithID(item, parent_id, 'waypoint_'+str(len(ret)))
                task.poses.append(self.earth.geoToPose(item['latitude'], item['longitude']))
                task.type = "goto"

            elif item['type'] == 'SurveyPattern' or item['type'] == 'SearchPattern':
                task = self.newTaskWithID(item, parent_id, 'area_'+str(len(ret['tasks'])))
                task.type = "survey_area"
                ret['tasks'].append(task)
                line_count = 1
                for c in item['children']:
                    if c['type'] == 'TrackLine':
                        sub_task = self.parseTrackLine(c, parent_id + task.id + '/', 'line_' + str(line_count))
                        line_count += 1
                        ret['tasks'].append(sub_task)

            elif item['type'] == 'TrackLine':
                ret['tasks'].append(self.parseTrackLine(item, parent_id, 'line_'+str(len(ret['tasks']))))

            elif item['type'] == 'SurveyArea':
                task = self.newTaskWithID(item, parent_id, 'area_'+str(len(ret['tasks'])))
                task.type = "survey_area"
                self.parseWaypoints(item['children'], task)
                ret['tasks'].append(task)

                sub_tasks = self.parseMission(item['children'], parent_id+task.id+'/', ['Waypoint', 'Behavior'])
                ret['tasks'] += sub_tasks['tasks']

            if item['type'] == 'Group':
                task = self.newTaskWithID(item, parent_id, 'group_'+str(len(ret['tasks'])))
                task.type = "group"
                group_items = self.parseMission(item['children'], parent_id+task.id+"/", ['Behavior'])
                ret['tasks'] += group_items['tasks']
            
            if item['type'] == 'Orbit':
                task = self.newTaskWithID(item, parent_id, 'orbit_'+str(len(ret['tasks'])))
                task.type = "orbit"
                data = {'speed':item['speed'], 'radius':item['radius'], 'safety_distance':item['safetyDistance']}
                task.data = yaml.safe_dumps(data)
                if len(item['targetFrame']):
                    target = PoseStamped()
                    target.pose.orientation.w = 1.0
                    target.header.frame_id = item['targetFrame']
                    task.poses.append(target)
                ret['tasks'].append(task)

            if item['type'] == 'Behavior' and not 'Behavior' in ignore_list:
                ret['behaviors'].append(self.parseBehavior(item))



        return ret
    
if __name__ == '__main__':
    rospy.init_node('MissionManager')

    mm = MissionManager()
    rospy.spin()

