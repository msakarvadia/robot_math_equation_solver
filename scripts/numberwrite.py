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

class Robot(object):

    def __init__(self):
        # initialize this node
        rospy.init_node('move_robot')

        # Get the saved q matrix
        self.q_learning_matrix = np.loadtxt(os.path.dirname(__file__) + "/q_learning_matrix.csv", delimiter=",", dtype=str)

        # Get the action matrix
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Get the states

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Keep track of the current state and action
        self.curr_state = 0
        self.curr_action = None
        self.curr_action_index = None

        #differentiate which image component is being searched for
        self.batonact = False
        self.artagact = False

        self.completed_iterations = 0

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        lin = Vector3()
        ang = Vector3()
        self.vel = Twist(linear=lin, angular=ang)

        # Define twist publisher to help the robot move
        self.cmdpublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.bridge = cv_bridge.CvBridge()

        # subscribe to the lidar scan from the robot
        self.scan = LaserScan()
        rospy.Subscriber("/scan", LaserScan, self.get_scan)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        rospy.sleep(2)

    def inv_kin(x, y, self):
        # lengths are predetermined
        # x is also already known -> white board (can use lidar?)
        # only y needed

        

    def run(self):

        # rospy.spin()


if __name__ == "__main__":
    robot = Robot()
    robot.run()