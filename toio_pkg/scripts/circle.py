#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import rosparam
from std_msgs.msg import Int32, Float32, ColorRGBA
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from visualization_msgs.msg import Marker
from toio_pkg.msg import *
from tf.transformations import euler_from_quaternion
from bluepy import btle
import math

class Circle(object):
    def __init__(self):
        self.pose1_sub = rospy.Subscriber('/toio1/pose', PoseStamped, self.pose1_callback)
        self.twist1_pub = rospy.Publisher("/toio1/cmd_vel", TwistStamped, queue_size=10)
        self.pose2_sub = rospy.Subscriber('/toio2/pose', PoseStamped, self.pose2_callback)
        self.twist2_pub = rospy.Publisher("/toio2/cmd_vel", TwistStamped, queue_size=10)
        self.pose3_sub = rospy.Subscriber('/toio3/pose', PoseStamped, self.pose3_callback)
        self.twist3_pub = rospy.Publisher("/toio3/cmd_vel", TwistStamped, queue_size=10)
        self.pose4_sub = rospy.Subscriber('/toio4/pose', PoseStamped, self.pose4_callback)
        self.twist4_pub = rospy.Publisher("/toio4/cmd_vel", TwistStamped, queue_size=10)
        self.vis_pub = rospy.Publisher("marker", Marker, queue_size = 10)
        self.last_pose1 = PoseStamped()
        self.last_pose2 = PoseStamped()
        self.last_pose3 = PoseStamped()
        self.last_pose4 = PoseStamped()
        self.start_time = rospy.get_time()
        self.radius = 0.15
        self.w = 1.0

        rospy.loginfo("wait 3s")
        rospy.sleep(3.0)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.timerCallback)

    def pose1_callback(self,data):
        self.last_pose1 = data

    def pose2_callback(self,data):
        self.last_pose2 = data

    def pose3_callback(self,data):
        self.last_pose3 = data

    def pose4_callback(self,data):
        self.last_pose4 = data

    def timerCallback(self,event):
        t = rospy.get_time() - self.start_time
        target1 = PoseStamped()
        target1.pose.position = self.lissajous(t, 0.0)
        target1.pose.orientation.w = 1.0

        target2 = PoseStamped()
        target2.pose.position = self.lissajous(t, 0.5)
        target2.pose.orientation.w = 1.0

        target3 = PoseStamped()
        target3.pose.position = self.lissajous(t, 1.0)
        target3.pose.orientation.w = 1.0

        target4 = PoseStamped()
        target4.pose.position = self.lissajous(t, 1.5)
        target4.pose.orientation.w = 1.0

        cmd1, marker1 = self.move_common("toio1", 1, target1.pose, self.last_pose1.pose)
        cmd2, marker2 = self.move_common("toio2", 2, target2.pose, self.last_pose2.pose)
        cmd3, marker3 = self.move_common("toio3", 3, target3.pose, self.last_pose3.pose)
        cmd4, marker4 = self.move_common("toio4", 4, target4.pose, self.last_pose4.pose)

        self.twist1_pub.publish(cmd1)
        self.twist2_pub.publish(cmd2)
        self.twist3_pub.publish(cmd3)
        self.twist4_pub.publish(cmd4)

        self.vis_pub.publish(marker1)
        self.vis_pub.publish(marker2)
        self.vis_pub.publish(marker3)
        self.vis_pub.publish(marker4)

    def circle(self, t, p):
        position = Point()
        position.x = 0.15 * math.cos(1.0 * t + 3.1415 * p)
        position.y = 0.15 * math.sin(1.0 * t + 3.1415 * p)
        return position

    def lissajous(self, t, p):
        position = Point()
        position.x = 0.22 * math.cos(0.25 * (t + 3.1415 * p))
        position.y = 0.22 * math.sin(0.50 * (t + 3.1415 * p))
        return position

    def move_common(self, frame_id, marker_id, target, pose):
        (dx, dyaw) = self.get_diff(target.position.x, target.position.y, pose.position.x, pose.position.y, self.get_yaw(pose.orientation))
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = frame_id
        cmd_vel.header.stamp = rospy.get_rostime()
        cmd_vel.twist.linear.x = 2.0 * dx
        cmd_vel.twist.angular.z = 3.0 * dyaw
        
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = marker_id
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = target.position.x
        marker_data.pose.position.y = target.position.y
        marker_data.pose.position.z = 0.0
        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=0.0
        marker_data.pose.orientation.w=1.0
        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.05
        marker_data.scale.y = 0.02
        marker_data.scale.z = 0.02
        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0
        return cmd_vel, marker_data

    def get_diff(self, target_x , target_y, self_x, self_y, self_yaw):
        w_dx = target_x - self_x
        w_dy = target_y - self_y
        self_dx = w_dx * math.cos(-self_yaw) - w_dy * math.sin(-self_yaw)
        self_dy = w_dx * math.sin(-self_yaw) + w_dy * math.cos(-self_yaw)
        # print("self_dx, self_dy", self_dx, self_dy)
        return self_dx, math.atan2(self_dy, self_dx)

    def get_yaw(self, ori):
        angles = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return angles[2]

if __name__ == '__main__':
    rospy.init_node('toio_circle')
    try:
        Circle()
        rospy.spin()
    except rospy.ROSInterruptException: pass