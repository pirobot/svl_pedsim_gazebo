#!/usr/bin/env python

""" 
    simulator.py - Version 1.0 2018-01-05

    Pedestrian simulator for svl_pedsim_gazebo
"""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from svl_pedsim.agent import Agent

class Simulator():
    def __init__(self):
        
        agent1 = Agent()
        agent2 = Agent(x=2, y=2)
        agent1.spawn()
        agent2.spawn()
        

if __name__ == '__main__':
    try:
        Simulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("SVL pedsim simulator terminated.")
