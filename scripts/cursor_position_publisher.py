#!/usr/bin/env python3

import cv2
import numpy as np

import rospy, cv_bridge
from sensor_msgs.msg import Image
from robot_math_equation_solver.msg import CursorLocate


# Set tag for cursor
TAG_NUM = 1

class FindCursorTag:
    """
    Node that uses OpenCV to track the cursor location used for writing
    characters onto a wall in front of the robot. The initial writing
    operation uses a tag to mark where the robot should start writing.
    """
    
    def __init__(self):
        rospy.init_node("robot_math_wall_cursor_position_publisher")

        # OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Initialize tag dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Publish angle (radians) to cursor tag
        self.cursor_pub = rospy.Publisher('/robot_math/cursor_position', CursorLocate, queue_size=10, latch=True)

        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

        self.tag_loc = 0

    def image_callback(self, data):
        """
        Callback method which publishes the pixel location and image width of 
        the recognized cursor tag.
        """

        # Encode/preprocess image for processing with cv2
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Locate the tag location in image frame and publish if found
        corners, ids, _ = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

        if corners and np.any(ids == TAG_NUM):
            tag_idx = np.where(ids == TAG_NUM)[0][0]
            right_side  = corners[tag_idx][0][1][0]
            self.tag_loc = int(right_side)

        self.cursor_pub.publish(cursor_loc=self.tag_loc, image_width=data.width)


if __name__ == "__main__":
    node = FindCursorTag()
    rospy.spin()