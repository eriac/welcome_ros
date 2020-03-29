#!/usr/bin/env python
import rospy
import rosparam
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PoseConverter:
    def __init__(self):
        self.pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)

    def odomCallback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('pose_converter')
    try:
        PoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException: pass