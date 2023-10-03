#!/usr/bin/env python3

from typing import List, Dict

import rospy

from project11_msgs.msg import BehaviorInformation
from project11_nav_msgs.msg import TaskInformation, TaskFeedback

#from mission_manager.srv import TaskInformationIdList, TaskInformationIdListResponse, TaskInformationList, TaskInformationListResponse, TaskManagerCmd, TaskManagerCmdResponse
from mission_manager.srv import TaskManagerCmd, TaskManagerCmdResponse

from project11_navigation import Task, TaskList

import actionlib
import project11_navigation.msg

from mission_manager import camp_interface


class MissionManager(object):
    def __init__(self):

        """ 
        List of tasks to do, or already done.
        Keeping all the tasks allows us to run them in a loop
        """

        self.tasks = TaskList()


        # A task that may be added, such as hover,
        # to temporarily interrupt current task.
        self.override_task_id: str = None

        self.done_task_information = TaskInformation()
        self.done_task_information.type = "hover"
        self.done_task_information.id = "done_hover"
        self.done_task_information.priority = 100

        self.tasks.addOrUpdate(self.done_task_information)




        # A place to hold the running behavior information publishers.
        self.behavior_library: Dict[str, List[BehaviorInformation]] = {}
        self.behavior_info_publishers: Dict[str, rospy.Publisher] = {}
        self.behavior_feedback_subscribers: Dict[str, rospy.Subscriber] = {}

        self.navigator_client = actionlib.SimpleActionClient('navigator/run_tasks', 
                                                             project11_navigation.msg.RunTasksAction)
        
        self.taskServiceServer = rospy.Service('~task_manager',
                                               TaskManagerCmd,
                                               self.taskManagerCallback)
        

        self.camp = camp_interface.CampInterface(self)

        self.updateNavigator()



    def updateNavigator(self):
        goal = project11_navigation.msg.RunTasksGoal()
        goal.tasks = self.tasks.listMessages()

        for t in goal.tasks:

            # Populate the library with the behaviors for this task.
            self.behavior_library[t.id] = t.behaviors

            for bhv in t.behaviors:
                # Don't make new pub/sub's if they already exist.
                if bhv.id not in self.behavior_info_publishers.keys():
                    # Create behavior publisher and subscriber
                    self.behavior_info_publishers[bhv.id] = rospy.Publisher('project11/behaviors/' +
                                                                        bhv.type + 
                                                                        '/input',
                                                                        TaskFeedback,
                                                                        queue_size=10,
                                                                        latch=True)
            
                    self.behavior_feedback_subscribers[bhv.id] = rospy.Subscriber('project11/behaviors/' +
                                                                        bhv.type +
                                                                        '/feedback',
                                                                        BehaviorInformation,
                                                                        queue_size=1)
                #print("publishing behavior: %s" % bhv)
                #self.behavior_info_publishers[bhv.id].publish(t)

            # Send the behavior info here for each? or Wait until feedback from the navigator?
            


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


    def navigatorFeedbackCallback(self, feedback: project11_navigation.msg.RunTasksFeedback):

        # This block updates our local list of tasks with the status from the navigator.
        needUpdate = False
        if feedback is not None:
            task_ids = self.tasks.addOrUpdateMany(feedback.feedback.tasks)
            self.tasks.clearExcept(task_ids)

            for updated_task in feedback.feedback.tasks:
                if updated_task.id == self.override_task_id:
                    if updated_task.done:
                        self.tasks.remove(updated_task.id)
                        self.override_task_id = None
                        needUpdate = True

            # TODO: Verify that the task and its children activate the behavior. 
            if feedback.feedback.current_navigation_task in self.behavior_library.keys():
                task = self.tasks.task(feedback.feedback.current_navigation_task)
                while task is not None:
                    task_feedback = TaskFeedback()
                    task_feedback.current_navigation_task = feedback.feedback.current_navigation_task
                    task_feedback.tasks.append(task.message())
                    #task_feedback.tasks += task.desendantMessages()

                    for behavior in task.task_information.behaviors:
                        self.behavior_info_publishers[behavior.id].publish(task_feedback)
                    task = task.parent()

        self.camp.navigatorFeedback(feedback)

        if needUpdate:
            self.updateNavigator()

    def navigatorDoneCallback(self, state, result):
        self.camp.navigatorDone(state, result)

    
    def behaviorFeedbackCallback(self,feedback):
        rospy.loginfo('behavior feedback:', feedback)

    def replaceTasks(self, new_tasks):
        self.tasks.clear()
        self.tasks.addOrUpdateMany(new_tasks)
        self.updateNavigator()

    def appendTasks(self, new_tasks):
        self.tasks.addOrUpdateMany(new_tasks)
        self.updateNavigator()

    def prependTasks(self, new_tasks):
        self.addOrUpdateMany(new_tasks, prepend=True)
        self.updateNavigator()

    def clearTasks(self):
        self.tasks.clear()
        if self.done_task_information is not None:
            self.appendTasks([self.done_task_information,])
        self.updateNavigator()

    def updateTasks(self, new_or_updated_tasks):
        self.tasks.addOrUpdateMany(new_or_updated_tasks)
        self.updateNavigator()

    def setOverrideTask(self, task = None):
        if self.override_task_id is not None:
            self.tasks.remove(self.override_task_id)
        if task is None:
            self.override_task_id = None
        else:
            self.tasks.addOrUpdate(task, prepend=True)
            self.override_task_id = task.id
        self.updateNavigator()

    def updateLocalTaskList(self, command, new_tasks):
        '''Updates the local task list'''

        if command == 'replace_tasks':
            self.replaceTasks(new_tasks)
        elif command == 'append_tasks':
            self.appendTasks(new_tasks)
        elif command == 'prepend_tasks':
            self.prependTasks(new_tasks)
        elif command == 'clear_tasks':
            self.clearTasks()
        elif command == 'update':
            self.updateTasks(new_tasks)

        rospy.logwarn('mission_manager: Received unknown command on task_manager service: ' + command)

    def taskManagerCallback(self,msg):
        '''Receives commands to manipulate the navigator's task list.
        
        Args:
            string      command
            project11_nav_msgs.TaskInformation[] tasks
            ---
            string      result
            
        '''

        rospy.loginfo(msg)
        self.updateLocalTaskList(msg.command, msg.tasks)

        response = TaskManagerCmdResponse()
        return response

    
if __name__ == '__main__':
    rospy.init_node('mission_manager')

    mm = MissionManager()
    rospy.spin()

