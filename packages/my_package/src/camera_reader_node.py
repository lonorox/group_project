#!/usr/bin/env python3

import os

import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.publisherNode = rospy.Publisher("Red", Float64, queue_size=1)
        self.publisherNodeYellow = rospy.Publisher("Yellow", Float64, queue_size=1)
        self.publisherNodeWhite = rospy.Publisher("White", Float64, queue_size=1)


# 
    import cv2

    def detect_red_line(self, image):
        # Read the image

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color detection
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Mask image to only get red regions
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize total pixels and red pixels
        total_pixels = mask.shape[0] * mask.shape[1]
        red_pixels = 0

        # Iterate through contours and draw them on the original image
        for contour in contours:
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            # Count red pixels
            red_pixels += cv2.contourArea(contour)

        # Calculate percentage of red pixels
        red_percentage = (red_pixels / total_pixels) * 100


        return image, red_percentage

    def remove_top_portion(self, image, ratio=2 / 3):
        """
        Removes the top portion of an image based on a ratio.

        Args:
            image: A numpy array representing the image data.
            ratio: Ratio of the top portion to remove (default: 2/3).

        Returns:
            A new image with the top portion removed.
        """
        # Get image height
        height = image.shape[0]

        # Calculate the number of rows to remove from the top
        rows_to_remove = int(height * ratio)

        return image[rows_to_remove:, :]
    
    def detect_white_line(self, image):
        # Read the image

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color detection
        lower_white = np.array([0, 0, 185])
        upper_white  = np.array([179, 70, 255])

        # Mask image to only get red regions
        mask = cv2.inRange(hsv, lower_white, upper_white)


        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize total pixels and red pixels
        total_pixels = mask.shape[0] * mask.shape[1]
        red_pixels = 0

        # Iterate through contours and draw them on the original image
        for contour in contours:
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            # Count red pixels
            red_pixels += cv2.contourArea(contour)

        # Calculate percentage of red pixels
        red_percentage = (red_pixels / total_pixels) * 100


        return image, red_percentage
    def detect_yellow_line(self, image):
        # Read the image

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color detection
        lower_yellow = np.array([14, 95, 0])
        upper_yellow = np.array([104, 255, 255])

        # Mask image to only get red regions
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        # Create a mask to identify pixels within the specified HSV range

        # Apply the mask to get the image with only the target color

        black_background = np.zeros_like(image)

        # Apply the mask to the black background instead of the original image
        detected_image = cv2.bitwise_and(image, image, mask=mask)

        # Initialize total pixels and red pixelstarget_pixel_count = 
        total_pixels = mask.shape[0] * mask.shape[1]
        red_pixels = np.sum(mask)


        # Calculate percentage of red pixels
        red_percentage = (red_pixels / total_pixels) * 100


        return detected_image, red_percentage


    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        redness,percentage = self.detect_red_line(self.remove_top_portion(image))
        yellow,percYellow = self.detect_yellow_line(self.remove_top_portion(image))
        white,percWhite= self.detect_yellow_line(self.remove_top_portion(image))
        self.publisherNode.publish(percentage)
        self.publisherNodeYellow.publish(percYellow)
        self.publisherNodeWhite.publish(percWhite)
        print(percYellow)
        cv2.imshow(self._window, yellow)

        cv2.waitKey(1)

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()