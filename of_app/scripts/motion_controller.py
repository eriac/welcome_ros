#!/usr/bin/env python
import rospy
import rosparam
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from of_msgs.msg import MotionCommand
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import math

class MotionController:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_stamped", TwistStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.last_odom = None
        self.motion_command_sub = rospy.Subscriber("motion_command", MotionCommand, self.motionCommandCallback)
        self.last_motion_command = MotionCommand()
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timerCallback)
        self.command_enable = False
        self.command_stage = 0
        self.vis_pub = rospy.Publisher("marker", Marker, queue_size = 10)

    def motionCommandCallback(self, msg):
        self.last_motion_command = msg
        self.command_enable = True
        self.command_stage = 0

    def odomCallback(self, msg):
        self.last_odom = msg

    def timerCallback(self, event):
        if(self.last_odom is None or (not self.command_enable)):
            return

        if(self.last_motion_command.type == MotionCommand.TYPE_GOTO_FORWARD):
            finish, cmd_vel, marker = self.calcurateTwist(self.last_motion_command.goal, self.last_odom)
            self.cmd_vel_pub.publish(cmd_vel)
            self.vis_pub.publish(marker)
            if finish:
                self.command_enable = False

    def calcurateTwist(self, goal, odom):
        target_x = goal.position.x
        target_y = goal.position.y
        target_yaw = self.get_yaw(goal.orientation) 
        self_x = odom.pose.pose.position.x
        self_y = odom.pose.pose.position.y
        self_yaw = self.get_yaw(odom.pose.pose.orientation)
        (dx, dyaw) = self.get_diff(target_x, target_y, self_x, self_y, self_yaw)
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = "toio1"
        cmd_vel.header.stamp = rospy.get_rostime()
        finish = False
        if self.command_stage == 0:
            cmd_vel.twist.linear.x = 0
            cmd_vel.twist.angular.z = self.clip(3.0 * dyaw, -1.5, 1.5)
            if math.fabs(dyaw) < 0.5:
                self.command_stage = 1
        elif self.command_stage == 1:
            cmd_vel.twist.linear.x = self.clip(2.0 * dx, -0.2, 0.2)
            cmd_vel.twist.angular.z = self.clip(3.0 * dyaw, -1.5, 1.5)
            if math.fabs(dyaw) < 0.3 and dx < 0.05:
                self.command_stage = 2
        elif self.command_stage == 2:
            cmd_vel.twist.linear.x = 0
            cmd_vel.twist.angular.z = self.clip(3.0 * (target_yaw - self_yaw), -1.5, 1.5)
            if math.fabs(target_yaw - self_yaw) < 0.1:
                self.command_stage = 3
        elif self.command_stage == 3:
            cmd_vel.twist.linear.x = 0
            cmd_vel.twist.angular.z = 0
            finish = True
        # print(self.command_stage)

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = 1
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = target_x
        marker_data.pose.position.y = target_y
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

        return finish, cmd_vel, marker_data

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

    def clip(self, value, min, max):
        if value < min:
            return min
        elif value > max:
            return max
        else:
            return value

if __name__ == '__main__':
    rospy.init_node('motion_controller')
    try:
        MotionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass