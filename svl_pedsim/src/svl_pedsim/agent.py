#!/usr/bin/env python

""" 
    agent.py - Version 1.0 2018-01-05

    Agent object for svl_pedsim_gazebo 
"""

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import uuid
import os
from math import sin, cos, atan2, exp, sqrt, pow, log10

pause_physics = rospy.ServiceProxy('gazebo/pause_physics', Empty)
unpause_physics = rospy.ServiceProxy('gazebo/unpause_physics', Empty)

class Type:
    person   = 0
    robot    = 1
    obstacle = 2
    
class Model:
    person_1_walking_lidar  = "person_1_walking_lidar"
    person_2_walking_lidar  = "person_2_walking_lidar"
    person_3_walking_lidar  = "person_3_walking_lidar"
    person_4_walking_lidar  = "person_4_walking_lidar"

class Agent(object):
    def __init__(self, hack_index=1, x=0, y=0, z=0, theta=0, radius=0.5, mass=68.0, max_speed=1.34, fov=180, agent_type=Type.person, model=Model.person_1_walking_lidar):
        
        self.hack_index  = hack_index
        self.id          = self.create_id()
        self.model_state = ModelState()
        self.agent_type  = agent_type
        self.model       = model
        self.name        = self.model + "_" + str(self.id)
        self.max_speed   = max_speed
        self.fov         = fov
        self.radius      = radius
        self.mass        = mass
        
        self.x           = x
        self.y           = y
        self.z           = z
        self.theta       = theta
        self.force       = 0.0
        self.pose        = Pose(position=Point(x,y,z))
        self.velocity    = Twist()
        self.last_update = None
        
        quaternion = quaternion_from_euler(0, 0, theta)
        
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]
        
        self.model_state.model_name = self.name
        self.model_state.reference_frame = 'world'
        self.model_state.pose = self.pose
        
    def spawn(self):
        spawn_command = "rosrun gazebo_ros spawn_model -database " + self.model \
            + " -sdf -model " + self.name + " -x " + str(self.x) + " -y " \
            + str(self.y) + " -z " + str(self.z) + " -Y " + str(self.theta)

        pause_physics()
        os.system(spawn_command)
        unpause_physics()
        
        # Hack to assign laser topic name as hard coded in SDF model file
        topic_name = "/person" + "_" + str(self.hack_index) + "/scan"
        self.lidar_msg = LaserScan()
        self.lidar_sub = rospy.Subscriber(topic_name, LaserScan, self.lidar_callback)
        
    def set_goal(self, x, y, z=0):
        self.goal_x = x
        self.goal_y = y
        self.goal_z = z
        
    def compute_forces(self, sfm):
        # Goal force (attractive)
        distance_to_goal = sqrt(pow(self.goal_x - self.x, 2) + pow(self.goal_y - self.y, 2))
        theta_goal = atan2(self.goal_y - self.y, self.goal_x - self.x)
        
        # Constant goal force model
        if distance_to_goal < sfm.goal_tolerance:
            force_goal_x = 0.0
            force_goal_y = 0.0
        else:
            force_goal_x = sfm.k0_goal * cos(theta_goal)
            force_goal_y = sfm.k0_goal * sin(theta_goal)
        
        # Net obstacle force (repulsive)
        force_obstacle_x = 0
        force_obstacle_y = 0
        
        angle = self.lidar_msg.angle_min
                
        for distance in self.lidar_msg.ranges:
            # Exponential obstacle force model
            force_obstacle_x += exp(-sfm.k0_obstacle * (distance - self.radius)) * cos(angle + self.theta)
            force_obstacle_y += exp(-sfm.k0_obstacle * (distance - self.radius)) * sin(angle + self.theta)
            angle += self.lidar_msg.angle_increment
            
        # Net social force (repulsive)
        # TO DO
        
        # Net force in the frame attached to the pedestrian
        force_x = sfm.k1_goal * force_goal_x - sfm.k1_obstacle * force_obstacle_x
        force_y = sfm.k1_goal * force_goal_y - sfm.k1_obstacle * force_obstacle_y
        
        return (force_x, force_y)
        
    def move(self, force, sfm):
        force_x, force_y = force
        
        # Newton's Law
        acceleration_x = force_x / self.mass
        acceleration_y = force_y / self.mass

        # Update velocity
        self.velocity.linear.x += acceleration_x * sfm.delta_t
        self.velocity.linear.y += acceleration_y * sfm.delta_t
        
        # TO DO: Rotate model to face direction of motion
    
        # Compute distance traveled this time step
        self.x += 0.5 * acceleration_x * sfm.delta_t * sfm.delta_t
        self.y += 0.5 * acceleration_y * sfm.delta_t * sfm.delta_t
        
        # Update pose of this person
        self.pose.position = Point(self.x, self.y, self.z)
        
        quaternion = quaternion_from_euler(0, 0, self.theta)
        
        self.pose.orientation.x = quaternion[0]
        self.pose.orientation.y = quaternion[1]
        self.pose.orientation.z = quaternion[2]
        self.pose.orientation.w = quaternion[3]
        
        # Pose
        self.model_state.pose = self.pose

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        
    def create_id(self):
        return uuid.uuid4()
    