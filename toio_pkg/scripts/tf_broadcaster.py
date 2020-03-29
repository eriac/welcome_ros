#!/usr/bin/python
# -*- coding: utf-8 -*- 

import rospy
import rosparam
from std_msgs.msg import Int32, Float32, ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped
from toio_pkg.msg import *
from tf.transformations import euler_from_quaternion
from bluepy import btle
import math


#!/usr/bin/env python  
import rospy
import tf

class Broadcaster:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        
    def pose_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), self.frame_id, msg.header.frame_id)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    try:
        Broadcaster()
        rospy.spin()
    except rospy.ROSInterruptException: pass
