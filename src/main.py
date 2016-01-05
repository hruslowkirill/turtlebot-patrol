#!/usr/bin/env python

""" patrol_smach_concurrence.py - Version 1.0 2013-04-12

    Control a robot using SMACH to patrol around a square a specified number of times
    while monitoring battery levels using the Concurrence container.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import ar_track_alvar
from ar_track_alvar.msg import AlvarMarkers

from actions import *
from RobotPose import *
from KeyPointsManager import *
        

class MainSM():
    def __init__(self):
        rospy.init_node('final_project_kl', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        self.keyPointManager = KeyPointManager()
        
         # Create the nav_patrol state machine using a Concurrence container
        self.nav_patrol = Concurrence(outcomes=['succeeded', 'key_points', 'stop'],
                                        default_outcome='succeeded',
                                        outcome_map = {'key_points' : {'MONITOR_AR':'invalid'}},
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        # Add the sm_nav machine and a AR Tag MonitorState to the nav_patrol machine             
        with self.nav_patrol:
           Concurrence.add('SM_NAV', Patrol().getSM())
           Concurrence.add('MONITOR_AR',MonitorState('/ar_pose_marker', AlvarMarkers, self.ar_cb))
        
        
        # Create the top level state machine
        self.sm_top = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])   
        self.sm_top.userdata.sm_ar_tag = None

        with self.sm_top:
            StateMachine.add('PATROL', self.nav_patrol, transitions={'succeeded':'PATROL', 'key_points':'PATROL_KEYPOINTS', 'stop':'STOP'}) 
            StateMachine.add('PATROL_KEYPOINTS', PatrolThroughKeyPoints(self.keyPointManager).getSM(), transitions={'succeeded':'STOP'})
            StateMachine.add('STOP', Stop(), transitions={'succeeded':''}) 
            
            
        intro_server = IntrospectionServer('patrol', self.sm_top, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = self.sm_top.execute()
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()
        
        
    def concurrence_outcome_cb(self, outcome_map):
        if outcome_map['MONITOR_AR'] == 'invalid':
            return 'key_points'
        return 'succeeded'

    def concurrence_child_termination_cb(self, outcome_map):
        if outcome_map['MONITOR_AR'] == 'invalid':
            rospy.loginfo("Monitor AR invalid...")
            return True
        return False

    def ar_cb(self, userdata, msg):
        if len(msg.markers)>0:
            rospy.loginfo('Ar tag was read: ' + str(msg.markers[0].id))
            rospy.loginfo(self.keyPointManager)
            for i in range(0,len(msg.markers)):
                if (self.keyPointManager.markerHasValidId(msg.markers[i])):
                    self.keyPointManager.add(msg.markers[i]);
                    if (self.keyPointManager.keyPointListComplete()):
                        return False
                    rospy.loginfo('Ar tag with number '+str(msg.markers[i].id)+' is detected')
                    
        
        return True

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MainSM()
    except rospy.ROSInterruptException:
        rospy.loginfo("Prohram test finished.")
