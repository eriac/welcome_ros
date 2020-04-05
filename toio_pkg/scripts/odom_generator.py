#!/usr/bin/env python
import rospy
import rosparam
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class OdomGenerator:
    def __init__(self):
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.poseCallback)
        self.twist_sub = rospy.Subscriber("cmd_vel", TwistStamped, self.twistCallback)
        self.last_pose = PoseStamped()
        self.last_twist = TwistStamped()

    def poseCallback(self, msg):
        self.last_pose = msg
        self.generateOdom()

    def twistCallback(self, msg):
        self.last_twist = msg
        self.generateOdom()

    def generateOdom(self):
        output = Odometry()
        output.header.stamp = rospy.get_rostime()
        output.pose.pose = self.last_pose.pose
        (roll, pitch, yaw) = euler_from_quaternion((self.last_pose.pose.orientation.x, self.last_pose.pose.orientation.y, self.last_pose.pose.orientation.z, self.last_pose.pose.orientation.w))
        vel_x = math.cos(yaw) * self.last_twist.twist.linear.x
        vel_y = math.sin(yaw) * self.last_twist.twist.linear.x
        output.twist.twist.linear.x = vel_x
        output.twist.twist.linear.y = vel_y
        output.twist.twist.angular = self.last_twist.twist.angular
        self.odom_pub.publish(output)

if __name__ == '__main__':
    rospy.init_node('odom_generator')
    try:
        OdomGenerator()
        rospy.spin()
    except rospy.ROSInterruptException: pass