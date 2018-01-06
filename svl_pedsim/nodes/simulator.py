#!/usr/bin/env python

""" 
    simulator.py - Version 1.0 2018-01-05

    Pedestrian simulator for svl_pedsim_gazebo
"""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from svl_pedsim.agent import Agent

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2




class Simulator():
    def __init__(self):
        
        rospy.init_node('svl_pedsim_simulator')
        
        #agent1 = Agent()
        #agent2 = Agent(x=2, y=2)
        #agent1.spawn()
        #agent2.spawn()
        
        self.manager = yield From(pygazebo.connect(('localhost', 11345)))

        self.pygz()

        rospy.spin()
        
    def pygz(self):

        
        self.manager.subscribe('/gazebo/default/physics/contacts',
                  'gazebo.msgs.GzString',
                  self.callback)
        
        
    def callback(data):
        message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
        print('Received message:', message.data)
        
if __name__ == '__main__':
    try:
        Simulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("SVL pedsim simulator terminated.")
