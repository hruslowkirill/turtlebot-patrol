#!/usr/bin/env python



import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
#from rbx2_tasks.task_setup import *
from math import copysign
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener
import ar_track_alvar
from ar_track_alvar.msg import AlvarMarkers
import numpy as np

from RobotPose import *
from task_setup import *
        
class Stop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Shutting down the state machine")
        return 'succeeded'
        
class SpinRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        pass
    
    def execute(self, userdata):
        rospy.loginfo("Spinning")
        self.r = rospy.Rate(10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.cmd_vel_pub.publish(Twist((0,0,0), (0,0,0.25)))
        counter = 100
        while counter>0:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            counter = counter-1
            self.r.sleep()
        return 'succeeded'
        
class PickWaypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'], input_keys=['waypoints'], output_keys=['waypoint_out'])
    
    def execute(self, userdata):   
        waypoint_out = randrange(len(userdata.waypoints))
        userdata.waypoint_out = userdata.waypoints[waypoint_out]
        rospy.loginfo("Going to waypoint " + str(waypoint_out))
        return 'succeeded'
        
class PickWaypointKeyPoint(State):
    def __init__(self, keyPointManager):
        State.__init__(self, outcomes=['succeeded'], input_keys=['waypoints'], output_keys=['waypoint_out'])
        self.keyPointManager = keyPointManager
        self.currentKeyPoint = 0
    def execute(self, userdata):   
        if (self.currentKeyPoint==len(self.keyPointManager.keyPointList)):
            self.currentKeyPoint = 0
        
        userdata.waypoint_out = self.keyPointManager.keyPointList[self.currentKeyPoint].pose
        rospy.loginfo("Going to keypoint " + str(self.keyPointManager.keyPointList[self.currentKeyPoint].id))
        self.currentKeyPoint = self.currentKeyPoint+1
    
        return 'succeeded'
        
class Nav2Waypoint(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'],
                       input_keys=['waypoint_in'])
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'

    def execute(self, userdata):
        self.goal.target_pose.pose = userdata.waypoint_in
    
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
        
        if self.preempt_requested():
            self.move_base.cancel_goal()
            self.service_preempt()
            return 'preempted'
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'aborted'
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            return 'succeeded'
        
class Patrol(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        setup_task_environment(self);
        # Initialize the navigation state machine
        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        self.sm_nav.userdata.waypoints = self.waypoints
        with self.sm_nav:            
            StateMachine.add('PICK_WAYPOINT', PickWaypoint(),
                             transitions={'succeeded':'NAV_WAYPOINT'},
                             remapping={'waypoint_out':'patrol_waypoint'})
            
            StateMachine.add('NAV_WAYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'PICK_WAYPOINT', 
                                          'aborted':'PICK_WAYPOINT', 
                                          'preempted':''},
                             remapping={'waypoint_in':'patrol_waypoint'})
        
        pass
    def move_base_result_cb(self, userdata, status, result):
        pass
    def execute(self, userdata):
        
        rospy.loginfo("Patroling")
        while 1>0:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        return 'succeeded'
        
    def getSM(self):
        return self.sm_nav
        
class PatrolThroughKeyPoints(State):
    def __init__(self, keyPointManager):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.keyPointManager = keyPointManager

        self.sm_nav = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        

        with self.sm_nav:            
            StateMachine.add('PICK_KEYPOINT', PickWaypointKeyPoint(self.keyPointManager),
                             transitions={'succeeded':'NAV_KEYPOINT'},
                             remapping={'waypoint_out':'patrol_waypoint'})
            
            StateMachine.add('NAV_KEYPOINT', Nav2Waypoint(),
                             transitions={'succeeded':'PICK_KEYPOINT', 
                                          'aborted':'PICK_KEYPOINT', 
                                          'preempted':''},
                             remapping={'waypoint_in':'patrol_waypoint'})
        
        pass
    def move_base_result_cb(self, userdata, status, result):
        pass
    def execute(self, userdata):
        self.sm_nav.userdata.waypoints = self.keyPointManager.getWaypoints()
        rospy.loginfo("Patroling throught keypoints")
        
        rospy.loginfo(self.keyPointManager)
        while 1>0:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        return 'succeeded'
    def getSM(self):
        return self.sm_nav

