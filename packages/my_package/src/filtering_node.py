#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


HOST = os.environ['VEHICLE_NAME']
class FilteringNode(DTROS):
    def __init__(self, node_name):
        super(FilteringNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()

        self.sub_cam = rospy.Subscriber(
            f'/{HOST}/camera_node/image/compressed',
            CompressedImage,
            self.cam_cb
        )

        self.pub_gauss = rospy.Publisher(
            '/gauss',
            CompressedImage,
            queue_size=1
        )

        self.pub_thresh = rospy.Publisher(
            '/thresh',
            CompressedImage,
            queue_size=1
        )

        self.pub_left = rospy.Publisher(
            '/left',
            CompressedImage,
            queue_size=1
        )

        self.pub_right = rospy.Publisher(
            '/right',
            CompressedImage,
            queue_size=1
        )

    def stmat_left(shape):
        mat = np.zeros(shape)
        mat[:, shape[1] // 3:] = -3
        return mat
    
    def stmat_right(shape):
        mat = np.zeros(shape)
        mat[:,:shape[1] * 2 // 3] = 2
        return mat

    def cam_cb(self, img):

        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(img, 'bgr8')
        img = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2GRAY)

        gauss = cv2.GaussianBlur(img_cv2, (0,0), 3)
        # self.publish_gauss(gauss)

        sobelx = cv2.Sobel(gauss, cv2.CV_64F, 1, 0)

        mag = cv2.Canny(gauss, 50, 150)

        width = mag.shape[1]
        left = np.ones(mag.shape)
        right = np.ones(mag.shape)

        left[:,int(np.floor(width/2)):width+1] = 0
        right[:, 0:int(np.floor(width/2))] = 0

 


        self.publish_left(mag * left)
        self.publish_right(mag * right)
        
        # ml = self.stmat_left(mag.shape) * (mag * left)
        # mr = self.stmat_right(mag.shape) * (mag * right)
        # s = np.sum(ml + mr)

        # rospy.loginfo(f'thinking omega={s}')
        # sobely = cv2.Sobel(gauss, cv2.CV_64F, 0, 1)

        # mag = np.sqrt(sobelx*sobelx + sobely*sobely)

        # thresh = cv2.threshold(mag, cv2.THRESH_BINARY)
        # self.publish_thresh(thresh)

        # width = thresh.shape[1]
        # mask_left = np.ones(sobelx.shape)
        # mask_left[:, int(np.floor(width/2)):width + 1] = 0

        # mask_right = np.ones(sobelx.shape)
        # mask_right[:, 0:int(np.floor(width/2))] = 0

        # self.publish_left(thresh, mask_left)
        # self.publish_right(thresh, mask_right)

    def publish_gauss(self, img):
        compressed = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_gauss.publish(compressed)

    def publish_thresh(self, img):
        comp = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_thresh.publish(comp)

    def publish_left(self, img):
        comp = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_left.publish(comp)

    def publish_right(self, img):
        comp = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_right.publish(comp)
        


if __name__ == '__main__':
    node = FilteringNode(node_name='filtering_node')
    rospy.spin()