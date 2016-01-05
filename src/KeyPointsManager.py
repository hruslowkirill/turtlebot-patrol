import rospy
import operator
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf import TransformListener
from visualization_msgs.msg import Marker
from KeyPoint import *
import numpy as np

class KeyPointManager(object):
    def __init__(self):
        self.tf = TransformListener()
        self.keyPointList = list()
        
    def add(self, marker):
        for i in range(len(self.keyPointList)):
            if (self.keyPointList[i].id==marker.id):
                return
        position = self.transformMarkerToWorld(marker)
        k = KeyPoint(marker.id, Point(position[0], position[1], position[2]), Quaternion(0.,  0.,  0.,  1.))
        self.keyPointList.append(k)
        self.addWaypointMarker(k)
        rospy.loginfo('Marker is added to following position')
        rospy.loginfo(k.pose)
        pass
    def getWaypoints(self):
        waypoints = list()
        for i in range(len(self.keyPointList)):
            waypoints.append(self.keyPointList[i].pose);
        return waypoints
            
    def keyPointListComplete(self):
        if (len(self.keyPointList)==5):
            self.keyPointList.sort(key=lambda x: x.id, reverse=True)
            return True
        return False
    def markerHasValidId(self, marker):
        if (marker.id>=61) and (marker.id<=65):
            return True
        return False
    def transformMarkerToWorld(self, marker):
        markerTag = "ar_marker_"+str(marker.id )
        rospy.loginfo(markerTag);
        if self.tf.frameExists("map") and self.tf.frameExists(markerTag):
            self.tf.waitForTransform("map", markerTag, rospy.Time(), rospy.Duration(4.0))
            t = self.tf.getLatestCommonTime("map", markerTag)
            position, quaternion = self.tf.lookupTransform("map", markerTag, t)
            return self.shiftPoint( position, quaternion)
    def shiftPoint(self, position, quaternion):
        try:
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            tf_mat = np.array([[np.cos(yaw), -np.sin(yaw), 0, position[0]],[np.sin(yaw), np.cos(yaw), 0, position[1]],[0, 0, 1, 0],[0, 0, 0, 1]])
            displacement = np.array([[1, 0, 0, 0],[0, 1, 0, 0.35],[0, 0, 1, 0],[0, 0, 0, 1]])
            point_map = np.dot(tf_mat, displacement)
            position = (point_map[0,3], point_map[1,3], 0)
        except Exception as inst:
            print type(inst)     # the exception instance
            print inst.args      # arguments stored in .args
            print inst
        return position
    def addWaypointMarker(self, keyPoint):
        rospy.loginfo('Publishing marker')
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = keyPoint.id
        marker_color = {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0}
        
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
        p = Point(keyPoint.pose.position.x, keyPoint.pose.position.y, keyPoint.pose.position.z)
        self.waypoint_markers.points.append(p)

        # Publish the waypoint markers
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)

        self.marker_pub.publish(self.waypoint_markers)

    
