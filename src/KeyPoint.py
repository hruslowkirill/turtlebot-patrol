import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

class KeyPoint(object):
    def __init__(self, id, position, quaternion):
        self.id = id
        self.pose = Pose(position, quaternion);
