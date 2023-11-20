#!/usr/bin/env python3

import cv2
import numpy as np

import rospy, cv_bridge
from sensor_msgs.msg import Image
from robot_math_equation_solver.msg import CursorLocate


TAG_NUM = 1
IMAGE_WIDTH = 320


class FindCursorTag:
    
    def __init__(self):
        rospy.init_node("robot_math_wall_cursor_locator")

        # OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Initialize tag dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # Publish angle (radians) to cursor tag
        self.cursor_pub = rospy.Publisher('/robot_math/cursor_locator', CursorLocate, queue_size=10, latch=True)

        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)


    def image_callback(self, data):
        # Pixel location of tag in image
        loc = -1

        # Encode/preprocess image for processing with cv2
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Locate the tag location in image frame and publish if found
        corners, ids, _ = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

        if corners and np.any(ids == TAG_NUM):
            tag_idx = np.where(ids == TAG_NUM)[0][0]
            right_corner  = corners[tag_idx][0][1][0]
            loc = int(right_corner)

        self.cursor_pub.publish(cursor_loc=loc, image_width=IMAGE_WIDTH)


if __name__ == "__main__":
    node = FindCursorTag()
    rospy.spin()