#!/usr/bin/env python3

import rospy

import moveit_commander

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointPres:
    
    def __init__(self):
        rospy.init_node('joint_2_pres')
        self.effort_sub = rospy.Subscriber("joint_states", JointState, self.joint_2_pressure_callback)
        self.adj_pub = rospy.Publisher("")

    def joint_2_pressure_callback(self, msg):
        cur_effort = msg.effort[3]        
        diff = 20 - cur_effort

        radian_adj = 0.05
        control_adj = radian_adj * diff


if __name__ == "__main__":
    node = JointPres()
    rospy.spin()