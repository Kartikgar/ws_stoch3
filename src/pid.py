#! /usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Controller():
    def __init__(self):
        
        rospy.init_node('PID_Controller')
        
        self.obj_pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pos_callback)
        self.goal_pos_sub = rospy.Subscriber('/goal', Odometry, self.goal_callback)
        self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        # x,y, theta, vx, vy, w
        # self.robot0_states = []
        self.goal_state=[0,0]
        self.robot_state = [0,0,0,0,0,0]
        self.Kp = 0.00656
        self.Kd = 0.0001
        self.Ki = 0.0
        self.dt = 0.5
        self.v = 0
        self.w = 0
        self.prev_heading_error =0
        self.total_heading_error =0
    def pos_callback(self, msg):

        # robot0_pos = msg.pose[2]
        # robot0_vel = msg.twist[2]
        robot1_pos = msg.pose[1]
        robot1_vel = msg.twist[1]

        # robot1_orientation = [robot0_pos.orientation.x,robot0_pos.orientation.y, robot0_pos.orientation.z, robot0_pos.orientation.w ]
        robot2_orientation = [robot1_pos.orientation.x, robot1_pos.orientation.y, robot1_pos.orientation.z, robot1_pos.orientation.w ]
        
        # _,_, theta1 = euler_from_quaternion(robot1_orientation)
        _,_, theta2 = euler_from_quaternion(robot2_orientation)

        # self.robot0_states = [robot0_pos.position.x, robot0_pos.position.y, theta1, robot0_vel.linear.x, robot0_vel.linear.y, robot0_vel.angular.z]
        self.robot_state = [robot1_pos.position.x, robot1_pos.position.y, theta2, robot1_vel.linear.x, robot1_vel.linear.y, robot1_vel.angular.z]
        print()
        print('robot states')
        # print(self.robot0_states)
        print(self.robot_state)

    def goal_callback(self, msg):

        goal_pos_x = msg.pose.pose.position.x
        goal_pos_y = msg.pose.pose.position.y
        self.goal_state=[goal_pos_x, goal_pos_y]



    def gtg(self):  
    #The Go to goal controller
    
        # global prev_heading_error
        # global total_heading_error   
    
    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
        delta_theta = (np.arctan2((self.goal_state[1] - self.robot_state[1]), (self.goal_state[0] - self.robot_state[0]))) - self.robot_state[2]
    #restrict angle to (-pi,pi)
        delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
    
    #Error is delta_theta in degrees
        e_new = ((delta_theta*180.0)/np.pi)
        e_dot = (e_new - self.prev_heading_error)/self.dt 
        self.total_heading_error = (self.total_heading_error + e_new)*self.dt
    #control input for angular velocity
        self.w = (self.Kp*e_new) + (self.Ki*self.total_heading_error) + (self.Kd*e_dot)
        self.prev_heading_error = e_new
  
    #find distance to goal
        d = np.sqrt(((self.goal_state[0] - self.robot_state[0])**2) + ((self.goal_state[1] - self.robot_state[1])**2))
    
    #velocity parameters
        distThresh = 0.1#mm
    
    #control input for linear velocity
        self.v = (0.24/1.5)*(np.arctan(d - distThresh))
    
    #request robot to execute velocity
        # return[V,W]
    def publish(self):
        cmd_vel = Twist()
        self.gtg()
        cmd_vel.linear.x = self.v
        cmd_vel.angular.z = self.w
        self.control_pub.publish(cmd_vel)



if __name__ == "__main__":

    controller_node = Controller()
    # rospy.spin()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        controller_node.publish()
        rate.sleep()




