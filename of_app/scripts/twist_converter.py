#!/usr/bin/env python
import rospy
import rosparam
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.cmd_vel_stamped_sub = rospy.Subscriber("cmd_vel_stamped", TwistStamped, self.twistCallback)

    def twistCallback(self, msg):
        self.cmd_vel_pub.publish(msg.twist)

if __name__ == '__main__':
    rospy.init_node('twist_converter')
    try:
        TwistConverter()
        rospy.spin()
    except rospy.ROSInterruptException: pass