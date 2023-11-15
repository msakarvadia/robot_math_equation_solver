#!/usr/bin/env python3

import rospy, rospkg, cv2, cv_bridge, numpy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import os
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__)

def inv_kin(x, y, z):
        l1 = 0.077
        l2 = 0.130
        l3 = 0.124 + 0.126 # (gripper length added on)
        # lengths are predetermined
        # order returned is t1 (lowest joint) - t3 (highest one) 
        # we're gonna work with the arm always level
        theta1 = math.atan2(y,x)

        #r2 = r3 - l4*math.cos()

        #theta3 = 

        c3 = ((x**2) + (y**2) + (z**2) - ((l1**2) + (l2**2) + (l3**2)) - ((2*l1)*(z-l1)))/(2*l2*l3)
        print("c3")
        print(c3)
        s3 = np.sqrt(1 - c3)
        theta3 = math.atan2(s3,c3)

        k1 = (c3*l3) + l2
        k2 = s3 * l3

        ap = (-2)*k1*(l1 - z)
        bp = (((2*k1)*(l1-z))**2) - (4)*(k1**2 + k2**2)*(z**2 + l1**2 - k2**2 - (2*z*l1))
        # math.atan2(((z - l1)(c1 - s1)), (x - y)) - math.atan2((s3*l3), ((c3*l3)+ l2))
        theta2 = math.asin((ap + np.sqrt(bp))/(2*(k1**2 + k2**2)))

        return theta1, theta2, theta3

class Robot(object):

    def __init__(self):
        # initialize this node
        rospy.init_node('move_robot')

       



        # initalize the debugging window
        # cv2.namedWindow("window", 1)

        lin = Vector3()
        ang = Vector3()
        self.vel = Twist(linear=lin, angular=ang)

        # Define twist publisher to help the robot move
        self.cmdpublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # subscribe to the robot's RGB camera data stream
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # self.bridge = cv_bridge.CvBridge()

        # subscribe to the lidar scan from the robot
        # self.scan = LaserScan()
        # rospy.Subscriber("/scan", LaserScan, self.get_scan)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        rospy.sleep(2)

    

    def writenum(self):
        
        #test
        t1, t2, t3 = inv_kin(0.2, 0.02, 0.02)

        print(t1)
        print(t2)
        print(t3)

        arm_joint_goal = [t1, t2, t3, 0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)




        

    def run(self):
        self.writenum()
        # rospy.spin()


if __name__ == "__main__":
    robot = Robot()
    robot.run()