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
        
        rospy.init_node('svl_pedsim_simulator')
        
        agent1 = Agent(x=2.0, y=7.6)
        agent1.spawn()
        
        while not rospy.is_shutdown():
            rospy.loginfo("COMPUTE!")
            agent1.compute_forces()
            rospy.sleep(2.0)
        
        
if __name__ == '__main__':
    try:
        Simulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("SVL pedsim simulator terminated.")
