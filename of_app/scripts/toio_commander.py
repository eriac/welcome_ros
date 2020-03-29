#!/usr/bin/env python
import rospy
import rosparam
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from of_msgs.msg import MotionCommand
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import math

class ToioCommander:
    def __init__(self):
        self.command_sub = rospy.Subscriber("toio_command", String, self.toioCommandCallback)
        self.motion_pub = rospy.Publisher("motion_command", MotionCommand, queue_size = 10)

    def toioCommandCallback(self, msg):
        if(msg.data=="G0"):
            motion_command = MotionCommand()
            motion_command.type = MotionCommand.TYPE_GOTO_FORWARD
            motion_command.goal.position.x = -0.15
            motion_command.goal.position.y = 0.15
            motion_command.goal.orientation.w = 1.0
            self.motion_pub.publish(motion_command)
        if(msg.data=="G1"):
            motion_command = MotionCommand()
            motion_command.type = MotionCommand.TYPE_GOTO_FORWARD
            motion_command.goal.position.x = 0.15
            motion_command.goal.position.y = 0.0
            motion_command.goal.orientation.w = 1.0
            self.motion_pub.publish(motion_command)

if __name__ == '__main__':
    rospy.init_node('toio_commander')
    try:
        ToioCommander()
        rospy.spin()
    except rospy.ROSInterruptException: pass