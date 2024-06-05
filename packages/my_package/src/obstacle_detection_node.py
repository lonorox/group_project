#!/usr/bin/env python3

import numpy as np
import cv2
import os
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from duckietown.dtros import NodeType, DTROS

HOST = os.environ['VEHICLE_NAME']
class ObstacleDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ObstacleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # for converting from CompressedImage to cv2 and vice versa
        self.bridge = CvBridge()

        # subscribing to topic TOPIC_NAME, messaging object type is CompressedImage, on each notify callback is called
        self.sub = rospy.Subscriber(
            f'/{HOST}/camera_node/image/compressed',
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size="10MB"
        )

        # publishing to the new topic 'image_pub', messaging object type is CompressedImage
        self.pub = rospy.Publisher('image_pub', CompressedImage, queue_size=1)

        self.remote_pub = rospy.Publisher('obstacle_detected', Bool, queue_size=1)

    def callback(self, msg):

        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # vehicle mask
        is_detected, _ = cv2.findCirclesGrid(img_cv2, patternSize=(7, 3), flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=cv2.SimpleBlobDetector_create())

        # convert filtered result to CompressedImage
        # img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_cv2)

        # publish to 'image_pub'
        # self.pub.publish(img_filtered_compressed)
        self.remote_pub.publish(is_detected)

if __name__ == '__main__':
    node = ObstacleDetectionNode(node_name='obstacle_detection_node')
    rospy.spin()
