import rospy
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
from math import copysign
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import ar_track_alvar
from ar_track_alvar.msg import AlvarMarkers
import numpy as np

class RobotPose(object):
    def __init__(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.monitor_pose)
    def getPosition(self):
        return self.pose
    def getEuler(self):
        quaternion = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        return euler_angles
    def getTransformation(self):
        
        trans = self.pose.position
        euler_angles = self.getEuler()
        tf_mat = np.array([[np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0, trans.x], [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0, trans.y], [0, 0, 1, trans.z], [0, 0, 0, 1]])
        return tf_mat
    def monitor_pose(self, data):
        self.pose = data.pose.pose
    
RobotPoseGlobal = RobotPose();
