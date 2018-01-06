#!/usr/bin/env python

""" 
    agent.py - Version 1.0 2018-01-05

    Agent object for svl_pedsim_gazebo 
"""

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_ros import gazebo_interface as gazebo
import uuid
import os

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
        
    def set_goal(self, x, y, z=0):
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        
    def sum_forces(self):
        pass
        
    def create_id(self):
        return uuid.uuid4()
    