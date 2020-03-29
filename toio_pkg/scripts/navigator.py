#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import rosparam
from std_msgs.msg import Int32, Float32, ColorRGBA, String
from geometry_msgs.msg import Point, TwistStamped, Pose, PoseStamped
from visualization_msgs.msg import Marker
from toio_pkg.msg import *
from tf.transformations import euler_from_quaternion
from bluepy import btle
import math

class NavigatorMode:
    def __init__(self):
        self.mode = "none"
        # none: stop 
        # pose: target pose
        # motion: motion
        self.target_pose = Pose()
        self.speed = 0.03
        self.motion_id = 0

class Navigator(object):
    def __init__(self):
        self.target_pose_sub = rospy.Subscriber('target_pose', PoseStamped, self.target_pose_callback)
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.command_sub = rospy.Subscriber('command', String, self.command_callback)
        self.twist_pub = rospy.Publisher("cmd_vel_stamped", TwistStamped, queue_size=10)
        self.last_pose = PoseStamped()
        self.start_time = rospy.get_time()
        self.navigator_mode = NavigatorMode()
        self.frame_id = "toio1"
        self.marker_id = 1
        self.linear_th = 0.02
        self.angular_th = 0.08

        rospy.loginfo("wait 3s")
        rospy.sleep(3.0)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.timerCallback)

    def target_pose_callback(self,data):
        self.navigator_mode.mode = "pose"
        self.navigator_mode.target_pose = data.pose
        
    def command_callback(self,data):
        None

    def pose_callback(self,data):
        self.last_pose = data

    def timerCallback(self,event):
        if(self.navigator_mode.mode == "pose"):
            print("pose mode")
            cmd, marker = self.navigator_pose()
            self.twist_pub.publish(cmd)
            # self.vis_pub.publish(marker)
        else:
            cmd = TwistStamped()
            cmd.header.frame_id = self.frame_id
            self.twist_pub.publish(cmd)

    def navigator_pose(self):
        print(self.navigator_mode.target_pose, self.last_pose.pose)
        (dx, dyaw) = self.get_diff(self.navigator_mode.target_pose, self.last_pose.pose)
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = self.frame_id
        cmd_vel.header.stamp = rospy.get_rostime()

        if abs(dx) > self.linear_th:
            cmd_vel.twist.linear.x = 2.0 * dx
            cmd_vel.twist.angular.z = 3.0 * dyaw
        else:
            to = self.navigator_mode.target_pose.orientation
            ori_length = to.x * to.x + to.y * to.y + to.z * to.z + to.w * to.w
            if  ori_length < 0.9 or 1.1 < ori_length:
                print("out1", ori_length)
                self.navigator_mode.mode = "none"
            else:
                t_yaw = self.get_yaw(to)
                c_yaw = self.get_yaw(self.last_pose.pose.orientation)
                dyaw2 = math.sin(t_yaw - c_yaw)
                if abs(math.sin(dyaw2)) < self.angular_th:
                    print("out2", dyaw2)
                    self.navigator_mode.mode = "none"
                else:
                    cmd_vel.twist.angular.z = 3.0 * dyaw2

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = self.marker_id
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = self.navigator_mode.target_pose.position.x
        marker_data.pose.position.y = self.navigator_mode.target_pose.position.y
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

    def get_diff(self, target_pose, self_pose):
        w_dx = target_pose.position.x - self_pose.position.x
        w_dy = target_pose.position.y - self_pose.position.y
        self_yaw = self.get_yaw(self_pose.orientation)
        self_dx = w_dx * math.cos(-self_yaw) - w_dy * math.sin(-self_yaw)
        self_dy = w_dx * math.sin(-self_yaw) + w_dy * math.cos(-self_yaw)
        # print("self_dx, self_dy", self_dx, self_dy)
        return self_dx, math.atan2(self_dy, self_dx)

    def get_yaw(self, ori):
        angles = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        return angles[2]

if __name__ == '__main__':
    rospy.init_node('toio_navigator')
    try:
        Navigator()
        rospy.spin()
    except rospy.ROSInterruptException: pass