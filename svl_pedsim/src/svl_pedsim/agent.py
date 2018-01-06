#!/usr/bin/env python

""" 
    agent.py - Version 1.0 2018-01-05

    Agent object for svl_pedsim_gazebo 
"""

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_ros import gazebo_interface as gazebo
import uuid
import os
from math import sin, cos, atan2, pi

#rospy.wait_for_service('gazebo/pause_physics', Empty)
#rospy.wait_for_service('gazebo/unpause_physics', Empty)

physics = gazebo.ODEPhysics()

pause_physics = rospy.ServiceProxy('gazebo/pause_physics', Empty)
unpause_physics = rospy.ServiceProxy('gazebo/unpause_physics', Empty)

class Type:
    person   = 0
    robot    = 1
    obstacle = 2
    
class Model:
    person_walking_lidar  = "person_walking_lidar"
    person_walking_rgbd   = "person_walking_rgbd"
    person_standing_lidar = "person_standing_lidar"
    person_standing_rgbd  = "person_standing_rgbd"

class Agent(object):
    def __init__(self, x=0, y=0, z=0, max_speed=1.34, fov=180, agent_type=Type.person, model=Model.person_walking_lidar):
        
        self.id         = self.create_id()
        self.agent_type = agent_type
        self.model      = model
        self.max_speed  = max_speed
        self.fov        = fov
        self.x          = x
        self.y          = y
        self.z          = z
        
    def spawn(self):
        spawn_command = "rosrun gazebo_ros spawn_model -database " + self.model \
            + " -sdf -model " + self.model + "_" + str(self.id) + " -x " + str(self.x) + " -y " + str(self.y) + " -z " + str(self.z)

        pause_physics()
        os.system(spawn_command)
        unpause_physics()
        
        self.lidar_msg = LaserScan()
        self.lidar_sub = rospy.Subscriber("/person/scan", LaserScan, self.lidar_callback)
        
    def set_goal(self, x, y, z=0):
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        
    def lidar_callback(self, msg):
        self.lidar_msg = msg
        
    def compute_forces(self):
        sum_x = 0
        sum_y = 0
        
        angle = self.lidar_msg.angle_max
        
        count = 0
        
        #print self.lidar_msg.ranges
        
        for distance in self.lidar_msg.ranges:
            sum_x += distance * cos(angle)
            sum_y += distance * sin(angle)
            if count == 0:
                direction = atan2(sum_y, sum_x)
                print "FIRST DIRECTION", direction
            angle -= self.lidar_msg.angle_increment
            count = 1
        
        direction = atan2(sum_y, sum_x)
        print "Direction", angle, direction
        
    def create_id(self):
        return uuid.uuid4()
    