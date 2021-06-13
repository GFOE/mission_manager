#!/usr/bin/env python

from __future__ import print_function
from builtins import str
from builtins import object
import rospy
import smach
import smach_ros

from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry

from dynamic_reconfigure.server import Server
from mission_manager.cfg import mission_managerConfig

from dubins_curves.srv import DubinsCurvesLatLong
from dubins_curves.srv import DubinsCurvesLatLongRequest

import actionlib
import path_follower.msg
import path_planner.msg
import hover.msg
import manda_coverage.msg

import project11
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

import json
import math

class MissionManagerCore(object):
    def __init__(self):
        self.piloting_mode = 'standby'
        self.odometry = None

        self.tasks = [] # list of tasks to do, or already done. Keeping all the tasks allows us to run them in a loop
        #self.pending_tasks = [] # list of tasks to be done. Once a task is completed, it is dropped from this list. Overrides get prepended here.
        self.current_task = None
        self.override_task = None # a task that may be added, such as hover, to temporarily interupt current tast.
        self.saved_task = None # a task that was current when an override task was added
        self.pending_command = None

        self.lineup_distance = 25

        self.done_behavior = 'hover'
        
        rospy.Subscriber('project11/piloting_mode', String, self.pilotingModeCallback, queue_size = 1)
        rospy.Subscriber('odom', Odometry, self.odometryCallback, queue_size = 1)
        rospy.Subscriber('project11/mission_manager/command', String, self.commandCallback, queue_size = 1)
        rospy.Subscriber('project11/heartbeat', Heartbeat, self.heartbeatCallback, queue_size = 1)

        
        self.status_publisher = rospy.Publisher('project11/status/mission_manager', Heartbeat, queue_size = 10)
        self.endofline_publisher = rospy.Publisher('project11/endofline', String, queue_size = 1)

        self.config_server = Server(mission_managerConfig, self.reconfigure_callback)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def pilotingModeCallback(self, msg):
        self.piloting_mode = msg.data

    def heartbeatCallback(self, msg):
        for kv in msg.values:
            if kv.key == 'piloting_mode':
                self.piloting_mode = kv.value
            
    def odometryCallback(self, msg):
        self.odometry = msg
        
    def reconfigure_callback(self, config, level):
        self.waypointThreshold = config['waypoint_threshold']
        self.turnRadius = config['turn_radius']
        self.segmentLength = config['segment_length']
        self.default_speed = config['default_speed']
        if config['planner'] == 0:
            self.planner = 'path_follower'
        elif config['planner'] == 1:
            self.planner = 'path_planner'
        if config['done_behavior'] == 0:
            self.done_behavior = 'hover'
        elif config['done_behavior'] == 1:
            self.done_behavior = 'restart'
            
        return config
        
    def getPilotingMode(self):
        return self.piloting_mode
    
    def commandCallback(self, msg):
        parts = msg.data.split(None,1)
        cmd = parts[0]
        if len(parts) > 1:
            args = parts[1]
        else:
            args = None
                
        #print 'command:',cmd,'args:',args
        
        if cmd == 'replace_task':
            self.clearTasks()
            self.addTask(args)
            self.pending_command = 'next_task'
            
        if cmd == 'append_task':
            self.addTask(args)

        if cmd == 'prepend_task':
            self.addTask(args, True)
            
        if cmd == 'clear_tasks':
            self.clearTasks()
            
        if cmd in ('next_task','prev_task','goto_task','goto_line', 'start_line', 'restart_mission'):
            self.pending_command  = msg.data

        
        if cmd == 'override':
            parts = args.split(None,1)
            if len(parts) == 2:
                task_type = parts[0]   
                task = None
                if task_type == 'goto':
                    task = self.parseLatLong(parts[1])
                    if task is not None:
                        task['type'] = 'goto'
                if task_type == 'hover':
                    task = self.parseLatLong(parts[1])
                    if task is not None:
                        task['type'] = 'hover'
                if task is not None:
                    self.setOverride(task)

    def clearTasks(self):
        self.tasks = []
        self.current_task = None
        self.saved_task = None

    def addTask(self, args, prepend=False):
        parts = args.split(None,1)
        print(parts)
        if len(parts) == 2:
            task_type = parts[0]
            task = None
            if task_type == 'mission_plan':
                task = self.parseMission(parts[1])
            if task_type == 'goto':
                task = self.parseLatLong(args)
                if task is not None:
                    task['type'] = 'goto'
            if task_type == 'hover':
                task = self.parseLatLong(args)
                if task is not None:
                    task['type'] = 'hover'
            if task is not None: 
                if prepend:
                    self.tasks.insert(0,task)
                else:
                    self.tasks.append(task)
        print('tasks')
        print(self.tasks)

    def setOverride(self, task):
        self.override_task = task
        self.pending_command = 'do_override'
        
    def parseLatLong(self,args):
        latlon = args.split()
        if len(latlon) >= 2:
            try:
                lat = float(latlon[0])
                lon = float(latlon[1])
                return {'latitude':lat, 'longitude':lon}
            except ValueError:
                return None
        
    def parseMission(self, mp):
        ret = {'type':'mission_plan',
               'nav_objectives':[],
               'default_speed':self.default_speed,
               'do_transit':True
               }
        
        plan = json.loads(mp)
        
        for item in plan:
            print(item)
            if item['type'] == 'Platform':
                ret['default_speed'] = item['speed']*0.514444  # knots to m/s
            if item['type'] == 'SurveyPattern':
                for c in item['children']:
                    ret['nav_objectives'].append(c)
                ret['label'] = item['label']
            if item['type'] == 'TrackLine':
                ret['nav_objectives'].append(item)
                ret['label'] = item['label']
            if item['type'] == 'SurveyArea':
                ret['nav_objectives'].append(item)
                ret['label'] = item['label']
        
        ret['current_nav_objective_index'] = 0
        return ret

    def position(self):
      if self.odometry is not None:
        try:
          odom_to_earth = self.tfBuffer.lookup_transform("earth", self.odometry.header.frame_id, rospy.Time())
        except Exception as e:
          print(e)
          return
        ecef = do_transform_pose(self.odometry.pose, odom_to_earth).pose.position
        return project11.wgs84.fromECEFtoLatLong(ecef.x, ecef.y, ecef.z)

    def heading(self):
      if self.odometry is not None:
        o = self.odometry.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        return 90-math.degrees(euler_from_quaternion(q)[2])
      
    def distanceTo(self, lat, lon):
      p_rad = self.position()
      current_lat_rad = p_rad[0]
      current_lon_rad = p_rad[1]
      target_lat_rad = math.radians(lat)
      target_lon_rad = math.radians(lon)
      azimuth, distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, target_lon_rad, target_lat_rad)
      return distance

    def waypointReached(self, lat, lon):
      return self.distanceTo(lat, lon) < self.waypointThreshold

    def generatePathFromVehicle(self, targetLat, targetLon, targetHeading):
      p = self.position()
      h = self.heading()
      #print('generatePathFromVehicle',p,h)
      return self.generatePath(math.degrees(p[0]), math.degrees(p[1]), h, targetLat, targetLon, targetHeading)

    def generatePath(self, startLat, startLon, startHeading, targetLat, targetLon, targetHeading):
        #print('generatePath: from:',startLat,startLon,'to:',targetLat,targetLon)
        rospy.wait_for_service('dubins_curves_latlong')
        dubins_service = rospy.ServiceProxy('dubins_curves_latlong', DubinsCurvesLatLong)

        dubins_req = DubinsCurvesLatLongRequest()
        dubins_req.radius = self.turnRadius
        dubins_req.samplingInterval = self.segmentLength

        dubins_req.startGeoPose.position.latitude = startLat
        dubins_req.startGeoPose.position.longitude = startLon

        start_yaw = math.radians(self.headingToYaw(startHeading))
        start_quat = quaternion_from_euler(0.0,0.0,start_yaw)
        dubins_req.startGeoPose.orientation.x = start_quat[0]
        dubins_req.startGeoPose.orientation.y = start_quat[1]
        dubins_req.startGeoPose.orientation.z = start_quat[2]
        dubins_req.startGeoPose.orientation.w = start_quat[3]
        
        dubins_req.targetGeoPose.position.latitude = targetLat
        dubins_req.targetGeoPose.position.longitude = targetLon
      
        target_yaw = math.radians(self.headingToYaw(targetHeading))
        q = quaternion_from_euler(0.0,0.0,target_yaw)
        dubins_req.targetGeoPose.orientation.x = q[0]
        dubins_req.targetGeoPose.orientation.y = q[1]
        dubins_req.targetGeoPose.orientation.z = q[2]
        dubins_req.targetGeoPose.orientation.w = q[3]

        #print dubins_req
        dubins_path = dubins_service(dubins_req)
        #print (dubins_path)
        return dubins_path.path

    def segmentHeading(self,lat1,lon1,lat2,lon2):
        start_lat_rad = math.radians(lat1)
        start_lon_rad = math.radians(lon1)

        dest_lat_rad = math.radians(lat2)
        dest_lon_rad = math.radians(lon2)
        
        path_azimuth, path_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
        return math.degrees(path_azimuth)

    def headingToPoint(self,lat,lon):
      p = self.position()
      dest_lat_rad = math.radians(lat)
      dest_lon_rad = math.radians(lon)
      azimuth, distance = project11.geodesic.inverse(p[1], p[0], dest_lon_rad, dest_lat_rad)
      return math.degrees(azimuth)
    
    def headingToYaw(self, heading):
        return 90-heading

    def iterate(self, current_state):
        if rospy.is_shutdown():
            return 'exit'
        if self.getPilotingMode() != 'autonomous':
            return 'pause'
        if self.pending_command is not None:
            return 'cancelled'
        self.publishStatus(current_state)
        rospy.sleep(0.1)

    def nextTask(self):
        if self.pending_command is not None:
            print('nextTask: pending_command:',self.pending_command)
        if self.pending_command == 'do_override':
            if self.current_task is not None and self.current_task['type'] == 'mission_plan':
                self.current_task['current_path'] = None
            self.saved_task = self.current_task
            self.pending_command = None
            return
        
        if self.override_task is not None:
            self.current_task = self.saved_task
            if self.current_task is None and len(self.tasks):
                self.current_task = self.tasks[0]
            self.override_task = None
            if self.pending_command == 'next_task':
                self.pending_command = None
                return

        if self.pending_command == 'restart_mission' and len(self.tasks):
            for t in self.tasks:
                if t['type'] == 'mission_plan':
                    t['current_nav_objective_index'] = None
                    t['current_path'] = None
            self.current_task = self.tasks[0]

        if self.pending_command in ('next_task','prev_task'):
            if len(self.tasks):
                if self.current_task is None:
                    if self.pending_command == 'next_task':
                        self.current_task = self.tasks[0]
                    if self.pending_command == 'prev_task':
                        self.current_task = self.tasks[-1]
                else:
                    try:
                        i = self.tasks.index(self.current_task)
                        print('nextTask: current task index:',i)
                        if self.pending_command == 'next_task':
                            i += 1
                            if i >= len(self.tasks):
                                self.current_task = None
                            else:
                                self.current_task = self.tasks[i]
                        if self.pending_command == 'prev_task':
                            i -= 1
                            if i < 0:
                                self.current_task = None
                            else:
                                self.current_task = self.tasks[i]
                    except ValueError:
                        print("nextTask: can't find current task index!")
                        self.current_task = None
                    if self.current_task is None: #end of the list or error figuring out where in the list we were.
                        if self.done_behavior == 'restart':
                            self.current_task = self.tasks[0]
                        elif self.done_behavior == 'hover':
                            self.current_task = {'type':'hover'}
                            position = self.position()
                            self.current_task['latitude'] = math.degrees(position[0])
                            self.current_task['longitude'] = math.degrees(position[1])
                if self.current_task is not None and self.current_task['type'] == 'mission_plan':
                    self.current_task['current_nav_objective_index'] = None
                    self.current_task['current_path'] = None

                    
        if self.current_task is not None and self.current_task['type'] == 'mission_plan' and (self.pending_command is not None and ( self.pending_command.startswith('goto_line') or self.pending_command.startswith('start_line'))):
            parts = self.pending_command.strip().split(None,1)
            if len(parts) == 2:
                cmd = parts[0]
                line_no = int(parts[1])
                if line_no >= 0 and line_no < len(self.current_task['nav_objectives']):
                    self.current_task['current_nav_objective_index'] = line_no
                    self.current_task['current_path'] = None
                    if cmd == 'goto_line':
                        self.current_task['do_transit'] = False
                    if cmd == 'start_line':
                        self.current_task['do_transit'] = True
                
        
        self.pending_command = None

    def getCurrentTask(self):
        if self.override_task is not None:
            return self.override_task
            
        return self.current_task

    
    def publishStatus(self, state):
        hb = Heartbeat()
        hb.header.stamp = rospy.Time.now()

        hb.values.append(KeyValue('state',state))
        hb.values.append(KeyValue('tasks_count',str(len(self.tasks))))
        for t in self.tasks:
            tstring = t['type']
            if t['type'] == 'mission_plan' and 'label' in t:
                tstring += ' ('+t['label']+')'
            hb.values.append(KeyValue('-task',tstring))

        if self.current_task is None:
            hb.values.append(KeyValue('current_task','None'))
        else:
            hb.values.append(KeyValue('current_task_type',self.current_task['type']))
            if self.current_task['type'] == 'mission_plan':
                hb.values.append(KeyValue('current_task_label',self.current_task['label']))
                hb.values.append(KeyValue('current_task_nav_objective_count',str(len(self.current_task['nav_objectives']))))
                hb.values.append(KeyValue('current_task_nav_objective_index',str(self.current_task['current_nav_objective_index'])))
        self.status_publisher.publish(hb)
               
class MMState(smach.State):
    '''
    Base state for Mission Manager states
    '''
    def __init__(self, mm, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.missionManager = mm
        
class Pause(MMState):
    """
    This state is for all top level piloting_mode other than autonomous
    """
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['resume','exit'])
        
    def execute(self, userdata):
        while self.missionManager.getPilotingMode() != 'autonomous':
            if rospy.is_shutdown():
                return 'exit'
            self.missionManager.publishStatus('Pause')
            rospy.sleep(0.1)
        return 'resume'

class Idle(MMState):
    """
    In autonomous mode, but with no pending tasks.
    """
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['exit','do-task','pause'])
        
    def execute(self, userdata):
        while True:
            ret = self.missionManager.iterate('Idle')
            if ret == 'cancelled':
                return 'do-task'
            if ret is not None:
                return ret
            if len(self.missionManager.tasks) != 0:
                return 'do-task'


class NextTask(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['idle','mission_plan','goto','hover'])
        
    def execute(self, userdata):
        self.missionManager.nextTask()
        if self.missionManager.override_task is not None:
            return self.missionManager.override_task['type']
        if self.missionManager.current_task is not None:
            return self.missionManager.current_task['type']
        return 'idle'

class Hover(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['cancelled', 'exit', 'pause', 'follow_path'])
        self.hover_client = actionlib.SimpleActionClient('hover_action', hover.msg.hoverAction)
        
    def execute(self, userdata):
        task = self.missionManager.getCurrentTask()
        if task is not None:
            if not self.missionManager.waypointReached(task['latitude'],task['longitude']):
                path = []
                p = self.missionManager.position()
                gp = GeoPose()
                gp.position.latitude = math.degrees(p[0])
                gp.position.longitude = math.degrees(p[1])
                path.append(gp)
                g = GeoPose()
                g.position.latitude = task['latitude']
                g.position.longitude = task['longitude']
                path.append(g)
                task['path'] = path
                task['path_type'] = 'transit'
                task['default_speed'] = self.missionManager.default_speed
                return 'follow_path'
            goal = hover.msg.hoverGoal()
            goal.target.latitude = task['latitude']
            goal.target.longitude = task['longitude']
            self.hover_client.wait_for_server()
            self.hover_client.send_goal(goal)
        while True:
            ret = self.missionManager.iterate('Hover')
            if ret is not None:
                self.hover_client.cancel_goal()
                return ret

class LineEnded(MMState):
    def __init__(self,mm):
        MMState.__init__(self, mm, outcomes=['mission_plan','next_item'])

    def execute(self, userdata):
        task = self.missionManager.getCurrentTask()
        if task is not None and task['type'] == 'mission_plan':
            if task['transit_path'] is not None:
                task['transit_path'] = None
                self.missionManager.endofline_publisher.publish("transit")
            else:
                task['current_path'] = None
                task['current_nav_objective_index'] += 1
                self.missionManager.endofline_publisher.publish("track")
            return 'mission_plan'
        self.missionManager.pending_command = 'next_task'
        return 'next_item'

        
class MissionPlan(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['follow_path','survey_area','done'])
        
    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None:
            if task['current_nav_objective_index'] is None:
                task['current_nav_objective_index'] = 0
            if task['current_nav_objective_index'] >= len(task['nav_objectives']):
                task['current_nav_objective_index'] = None
                self.missionManager.pending_command = 'next_task'
                return 'done'
            if task['nav_objectives'][task['current_nav_objective_index']]['type'] == 'SurveyArea':
                return 'survey_area'
            if not 'current_path' in task or task['current_path'] is None:
                self.generatePaths(task)
            return 'follow_path'
        return 'done'

    def generatePaths(self, task):
        path = []
        print(task['nav_objectives'])
        for p in task['nav_objectives'][task['current_nav_objective_index']]['waypoints']:
            #path.append((p['position']['latitude'],p['position']['longitude']))
            gp = GeoPose()
            gp.position.latitude = p['latitude']
            gp.position.longitude = p['longitude']
            path.append(gp)
        task['current_path'] = path
        # decide if we transit or start line
        task['transit_path'] = None
        if len(task['current_path']) >1:
            start_point = task['current_path'][0]
            next_point = task['current_path'][1]
            if task['do_transit'] and self.missionManager.distanceTo(start_point.position.latitude,start_point.position.longitude) > self.missionManager.waypointThreshold and self.missionManager.planner == 'path_follower':
                #transit
                segment_heading = self.missionManager.segmentHeading(start_point.position.latitude,start_point.position.longitude,next_point.position.latitude,next_point.position.longitude)
                pre_start = project11.geodesic.direct(math.radians(start_point.position.longitude), math.radians(start_point.position.latitude), math.radians(segment_heading+180), self.missionManager.lineup_distance)
                #print ('heading',segment_heading, 'start:', start_point.position, 'pre start:',  math.degrees(pre_start[1]), math.degrees(pre_start[0]))
                transit_path = self.missionManager.generatePathFromVehicle(math.degrees(pre_start[1]), math.degrees(pre_start[0]), segment_heading)
                transit_path.append(start_point)
                task['transit_path'] = transit_path
            task['do_transit'] = True
        
class Goto(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','follow_path'])
        
    def execute(self, userdata):
        task = self.missionManager.getCurrentTask()
        if task is not None:
            if self.missionManager.distanceTo(task['latitude'],task['longitude']) <= self.missionManager.waypointThreshold:
                self.missionManager.pending_command = 'next_task'
                return 'done'
            
            headingToPoint = self.missionManager.headingToPoint(task['latitude'],task['longitude'])
            path = self.missionManager.generatePathFromVehicle(task['latitude'],task['longitude'],headingToPoint)
            task['path'] = path
            task['path_type'] = 'transit'
            task['default_speed'] = self.missionManager.default_speed
            return 'follow_path'
        
class FollowPath(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','cancelled','exit','pause', 'hover'])
        self.path_follower_client = actionlib.SimpleActionClient('path_follower_action', path_follower.msg.path_followerAction)
        self.path_planner_client = actionlib.SimpleActionClient('path_planner_action', path_planner.msg.path_plannerAction)
        self.task_complete = False
        
    def execute(self, userdata):
        task = self.missionManager.getCurrentTask()
        if task is not None:
            if self.missionManager.planner == 'path_follower':
                goal = path_follower.msg.path_followerGoal()
            elif self.missionManager.planner == 'path_planner':   
                goal = path_planner.msg.path_plannerGoal()
            goal.path.header.stamp = rospy.Time.now()
            if task['type'] in ('goto', 'hover'):
                path = task['path']
            if task['type'] == 'mission_plan':
                if task['transit_path'] is not None:
                    path = task['transit_path']
                else:
                    path = task['current_path']
            for s in path:
                #print s
                gpose = GeoPoseStamped()
                gpose.pose = s
                goal.path.poses.append(gpose)
            goal.speed = task['default_speed']
            self.task_complete = False
            if self.missionManager.planner == 'path_follower':
                self.path_planner_client.cancel_goal()
                self.path_follower_client.wait_for_server()
                self.path_follower_client.send_goal(goal, self.path_follower_done_callback, self.path_follower_active_callback, self.path_follower_feedback_callback)
            elif self.missionManager.planner == 'path_planner':
                self.path_follower_client.cancel_goal()
                self.path_planner_client.wait_for_server()
                self.path_planner_client.send_goal(goal, self.path_follower_done_callback, self.path_follower_active_callback, self.path_follower_feedback_callback)

        while True:
            ret = self.missionManager.iterate('FollowPath')
            if ret is not None:
                if ret == 'cancelled':
                    if self.missionManager.planner == 'path_follower':
                        self.path_follower_client.cancel_goal()
                    elif self.missionManager.planner == 'path_planner':
                        self.path_planner_client.cancel_goal()
                return ret
            if self.task_complete:
                if task['type'] == 'hover':
                    return 'hover'
                return 'done'
            if task['type'] == 'hover' and self.missionManager.waypointReached(task['latitude'], task['longitude']):
              if self.missionManager.planner == 'path_follower':
                self.path_follower_client.cancel_goal()
              elif self.missionManager.planner == 'path_planner':
                self.path_planner_client.cancel_goal()
              return 'hover'

    def path_follower_done_callback(self, status, result):
        self.task_complete = True
    
    def path_follower_active_callback(self):
        pass
    
    def path_follower_feedback_callback(self, msg):
        task = self.missionManager.getCurrentTask()
        if task is not None:
            if 'path_type' in task and task['path_type'] == 'transit':
                pass
                #print(msg)
        pass

class SurveyArea(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','cancelled','exit','pause'])
        self.survey_area_client = actionlib.SimpleActionClient('survey_area_action', manda_coverage.msg.manda_coverageAction)
        self.task_complete = False

    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None:
            goal = manda_coverage.msg.manda_coverageGoal()
            for wp in task['nav_objectives'][task['current_nav_objective_index']]['children']:
                print(wp)
                gp = GeoPoint()
                gp.latitude = wp['latitude']
                gp.longitude = wp['longitude']
                goal.area.append(gp)
            goal.speed = task['default_speed']
            self.task_complete = False
            self.survey_area_client.wait_for_server()
            self.survey_area_client.send_goal(goal, self.survey_area_done_callback, self.survey_area_active_callback, self.survey_area_feedback_callback)

        while True:
            ret = self.missionManager.iterate('SurveyArea')
            if ret is not None:
                if ret == 'cancelled':
                    self.survey_area_client.cancel_goal()
                return ret
            if self.task_complete:
                return 'done'

    def survey_area_done_callback(self, status, result):
        self.task_complete = True
    
    def survey_area_active_callback(self):
        pass
    
    def survey_area_feedback_callback(self, msg):
        pass
    
    
def main():
    rospy.init_node('MissionManager')
    
    missionManager = MissionManagerCore()

    sm_top = smach.StateMachine(outcomes=['exit'])
    
    with sm_top:
        smach.StateMachine.add('PAUSE', Pause(missionManager), transitions={'resume':'AUTONOMOUS', 'exit':'exit'})
        
        sm_auto = smach.StateMachine(outcomes=['pause','exit'])
        
        with sm_auto:
            smach.StateMachine.add('IDLE', Idle(missionManager), transitions={'do-task':'NEXTTASK', 'pause':'pause'})
            smach.StateMachine.add('NEXTTASK', NextTask(missionManager), transitions={'idle':'IDLE', 'mission_plan':'MISSIONPLAN', 'hover':'HOVER', 'goto':'GOTO'})
            smach.StateMachine.add('HOVER', Hover(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK', 'follow_path':'FOLLOWPATH'})
            smach.StateMachine.add('MISSIONPLAN', MissionPlan(missionManager), transitions={'done':'NEXTTASK', 'follow_path':'FOLLOWPATH', 'survey_area':'SURVEYAREA'})
            smach.StateMachine.add('GOTO',Goto(missionManager), transitions={'done':'NEXTTASK', 'follow_path':'FOLLOWPATH'})
            smach.StateMachine.add('FOLLOWPATH', FollowPath(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK', 'done':'LINEENDED', 'hover':'HOVER'})
            smach.StateMachine.add('LINEENDED', LineEnded(missionManager), transitions={'mission_plan': 'MISSIONPLAN', 'next_item':'NEXTTASK'})
            smach.StateMachine.add('SURVEYAREA', SurveyArea(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK', 'done':'NEXTTASK'})

        smach.StateMachine.add('AUTONOMOUS', sm_auto, transitions={'pause':'PAUSE', 'exit':'exit'})
    
    sis = smach_ros.IntrospectionServer('mission_manager', sm_top, '/mission_manager')
    sis.start()                                                                            

    sm_top.execute()
    rospy.spin()
    
if __name__ == '__main__':
    main()
