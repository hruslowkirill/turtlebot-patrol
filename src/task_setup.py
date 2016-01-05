#!/usr/bin/env python

""" task_setup.py - Version 1.0 2013-12-20

    Set up a number of waypoints and a charging station for use with simulated tasks using
    SMACH and teer.

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
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import  pi
from collections import OrderedDict
from random import randrange



def setup_task_environment(self):
    
    # How long do we have to get to each waypoint?
    self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
    
    
    # Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
    # Wait up to 60 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(60))    
    
    rospy.loginfo("Connected to move_base action server")
  
    quaternions = list()
    quaternions.append(Quaternion(0.,  0.,  0.,  1.))
    quaternions.append(Quaternion(0.        ,  0.        ,  0.38268343,  0.92387953))
    quaternions.append(Quaternion(0.        ,  0.        ,  0.70710678,  0.70710678))
    quaternions.append(Quaternion( 0.        ,  0.        ,  0.92387953,  0.38268343))
    
    quaternions.append(Quaternion(0.,  0.,  1.,  0.))
    quaternions.append(Quaternion(0.        ,  0.        ,  0.92387953, -0.38268343))
    quaternions.append(Quaternion(0.        ,  0.        ,  0.70710678,  -0.70710678))
    quaternions.append(Quaternion(0.        ,  0.        ,  0.38268343,  -0.92387953))
    
    points = list()
    points.append(Point(0.0727, 0.0258, 0.0))
    points.append(Point(1.3743, -1.5364, 0.0))
    points.append(Point(2.77588256071, -1.45161086221, 0.0))
    points.append(Point(2.7901169437, -0.306092800817, 0.0))
    
    points.append(Point(0.416864927563, -0.934977045727, 0.0))
    points.append(Point(2.79503222276, -0.695678483756, 0.0))
    points.append(Point(1.71589395964, -0.698163749998, 0.0))
    points.append(Point(3.8997689308, -0.653732806946, 0.0))
    points.append(Point(3.99956372378, 0.100569070992, 0.0))
    points.append(Point(3.92454997612, 0.904279156201, 0.0))
    
    points.append(Point(2.88553229878, 1.00627506212, 0.0))
    points.append(Point(1.29568365858, 1.08293093937, 0.0))
    points.append(Point(0.47891207152, 0.699489209986, 0.0))
    points.append(Point(1.90865642878, -0.0372046250631, 0.0))
    # Create a list to hold the waypoint poses
    self.waypoints = list()
    for i in range(14):
        self.waypoints.append(Pose(points[i], quaternions[randrange(len(quaternions))]));
            
    # Initialize the waypoint visualization markers for RViz
    init_waypoint_markers(self)
    
    # Set a visualization marker at each waypoint        
    for waypoint in self.waypoints:           
        p = Point()
        p = waypoint.position
        self.waypoint_markers.points.append(p)
        
        
    # Publisher to manually control the robot (e.g. to stop it)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
    
    
    # Publish the waypoint markers
    self.marker_pub.publish(self.waypoint_markers)
    rospy.sleep(1)
    self.marker_pub.publish(self.waypoint_markers)
    
    rospy.sleep(1)

def init_waypoint_markers(self):
    # Set up our waypoint markers
    marker_scale = 0.2
    marker_lifetime = 0 # 0 is forever
    marker_ns = 'waypoints'
    marker_id = 0
    marker_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
    
    # Define a marker publisher.
    self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
    
    # Initialize the marker points list.
    self.waypoint_markers = Marker()
    self.waypoint_markers.ns = marker_ns
    self.waypoint_markers.id = marker_id
    self.waypoint_markers.type = Marker.CUBE_LIST
    self.waypoint_markers.action = Marker.ADD
    self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
    self.waypoint_markers.scale.x = marker_scale
    self.waypoint_markers.scale.y = marker_scale
    self.waypoint_markers.color.r = marker_color['r']
    self.waypoint_markers.color.g = marker_color['g']
    self.waypoint_markers.color.b = marker_color['b']
    self.waypoint_markers.color.a = marker_color['a']
    
    self.waypoint_markers.header.frame_id = 'map'
    self.waypoint_markers.header.stamp = rospy.Time.now()
    self.waypoint_markers.points = list()

