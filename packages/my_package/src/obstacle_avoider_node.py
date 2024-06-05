#!/usr/bin/env python3

import rospy
import os
import numpy as np
import cv2
import os
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_srvs.srv import EmptyResponse, Empty
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import String, Bool

HOST = os.environ['VEHICLE_NAME']
cmd = f'/{HOST}/car_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{HOST}/wheels_driver_node/wheels_cmd_executed'

class ObstacleAvoiderNode(DTROS):
    def __init__(self, node_name):
        super(ObstacleAvoiderNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.state = 0

        self.sub_detected = rospy.Subscriber(
            'obstacle_detected',
            Bool,
            self.detected_cb
        )
        
        self.pub_cmd = rospy.Publisher(
            cmd,
            Twist2DStamped,
            queue_size=1
        )

        self.pub_wh = rospy.Publisher(
           wheels_cmd_executed,
           WheelsCmdStamped,
           queue_size=1 
        )

        tw = self.get_twist(0.4, 0)
        wh = self.get_wheel(0.4, 0.4)
        
        rospy.Rate(0.5).sleep()
        self.pub_cmd.publish(tw)
        self.pub_wh.publish(wh)


    def detected_cb(self, b):
        if not b or self.state != 0:
            return
        
        self.state += 1
        wh = self.get_wheel(0.45, 0.45)
        tw = self.get_twist(0.4, -8)

        self.pub_wh.publish(wh)


        self.pub_cmd.publish(tw)
        rospy.Rate(2).sleep()

        tw = self.get_twist(0.4, 8)
        self.pub_cmd.publish(tw)
        rospy.Rate(2).sleep()

        tw = self.get_twist(0.4, -8)
        self.pub_cmd.publish(tw)
        rospy.Rate(4)

        rw = self.get_twist(0.4, 0)
        self.pub_cmd.publish(tw)
        rospy.Rate(2)

        wh = self.get_wheel(0,0)
        tw = self.get_twist(0,0)
        self.pub_cmd.publish(tw)
        self.pub_wh.publish(wh)

        self.state = 0



    def get_wheel(self, l, r):
        wheel = WheelsCmdStamped()
        wheel.vel_left = l
        wheel.vel_right = r
        return wheel

    def get_twist(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        return twist
        

if __name__ == '__main__':
    node = ObstacleAvoiderNode(node_name='obstacle_avoider_node')
    rospy.spin()