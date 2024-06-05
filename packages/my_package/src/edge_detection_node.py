#!/usr/bin/env python3

import os
import cv2
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'

class EdgeDetectionNode(DTROS):
    def __init__(self, node_name):
        
        super(EdgeDetectionNode, self).__init__(node_name=node_name,node_type=NodeType.PERCEPTION)

        # for converting from CompressedImage to cv2 and vice versa
        self.bridge = CvBridge()

        # subscribing to topic TOPIC_NAME, messaging object type is CompressedImage, on each notify callback is called
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")

        # publishing to the new topic 'image_pub', messaging object type is CompressedImage
        self.pub = rospy.Publisher('image_pub', CompressedImage, queue_size=1)

    def callback(self, msg):
        print(f'callback with type ${type(msg)}')

        # converting CompressedImage to cv2
        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # applying filter
        img_filtered = cv2.Canny(img_cv2, 50, 150)

        # converting filtered result to CompressedImage
        img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_filtered)
        
        # publishing to 'image_pub'
        self.pub.publish(img_filtered_compressed)


if __name__ == '__main__':
    node = EdgeDetectionNode(node_name='edge_detection_node')
    rospy.spin()
