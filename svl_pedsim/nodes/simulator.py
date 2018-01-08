#!/usr/bin/env python

""" 
    simulator.py - Version 1.0 2018-01-05

    Pedestrian simulator for svl_pedsim_gazebo
"""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from svl_pedsim.agent import Agent, Model

class SFM():
    def __init__(self, k0_obstacle=1.0, k1_obstacle=2.0, k0_social=1.5, k1_social=3.0, k0_goal=10, k1_goal=300.0, goal_tolerance=0.5, delta_t=0.1):
        self.k0_obstacle    = k0_obstacle
        self.k1_obstacle    = k1_obstacle
        self.k0_social      = k0_social
        self.k1_social      = k1_social
        self.k0_goal        = k0_goal
        self.k1_goal        = k1_goal
        self.goal_tolerance = goal_tolerance
        self.delta_t        = delta_t

class Simulator():
    def __init__(self):
        
        rospy.init_node('svl_pedsim_simulator')
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        
        # Track the Gazebo model states
        self.model_states = ModelStates()
        
        # Subscriber to monitor all model states
        rospy.wait_for_message("gazebo/model_states", ModelStates)
        rospy.Subscriber("gazebo/model_states", ModelStates, self.get_model_states)
        
        # Publisher for moving the people models
        model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=5)
        
        # Intialize the social forces model
        sfm = SFM(k0_obstacle=1.0, k1_obstacle=75.0, k0_social=1.5, k1_social=3.0, k0_goal=10.0, k1_goal=300.0, goal_tolerance=0.5, delta_t=1.0/rate)
        
        # Create the first pedestrian
        agent1 = Agent(hack_index=1, model=Model.person_1_walking_lidar, x=4.29, y=8.52, theta=0.0)
        agent1.spawn()
        
        # Create the second pedestrian
        agent2 = Agent(hack_index=2, model=Model.person_2_walking_lidar, x=15.0, y=8.48, theta=3.14)
        agent2.spawn()
        
        # Set the goal coordinates for each person
        agent1.set_goal(15.0, 8.48359, 0.0)
        agent2.set_goal(4.29, 8.52, 0.0)
        
        while not rospy.is_shutdown():
            force1 = agent1.compute_forces(sfm)
            agent1.move(force1, sfm)
            model_state_pub.publish(agent1.model_state)
            
            force2 = agent2.compute_forces(sfm)
            agent2.move(force2, sfm)
            model_state_pub.publish(agent2.model_state)
            
            r.sleep()
            
    def get_model_states(self, msg):
        self.model_states = msg
        
if __name__ == '__main__':
    try:
        Simulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("SVL pedsim simulator terminated.")
