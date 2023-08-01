#!/usr/bin/env 

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import numpy as np
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import matplotlib.pyplot as plt
# import torch 
from sensor_msgs.msg import Image
# import cv2 as cv
# from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelStates
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GoalPublisher():
    def __init__(self):
        
        rospy.init_node('CBF_Controller')
        
        self.obj_pos_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, self.pos_callback)
        self.goal_pub = rospy.Publisher('/goal', Odometry, queue_size=3)

        # x,y, theta, vx, vy, w
        self.goals = [[3.07, -2.86],[3.50, -5.45], [1.59, -6.57], [-2.55, -5.97], [-2.45, -3.51], [-1.84, -2.61]]
        self.goal = 0
        self.thres = 0.2
        self.tick = 0


    def pos_callback(self, msg):

        robotx, roboty = msg.pose[1].position.x, msg.pose[1].position.y 
        # robot_vel = msg.twist[2]
        # robot0_orientation = [robot0_pos.orientation.x, robot0_pos.orientation.y, robot0_pos.orientation.z, robot0_pos.orientation.w ]        
        # _,_, theta1 = euler_from_quaternion(robot0_orientation)
        

        goalx, goaly = self.goals[self.goal]

        if self.tick % 25 == 0:
            print(f"robotpos: {robotx, roboty}")
            print(f"goalpos: {goalx, goaly}")

        self.tick += 1
        if np.sqrt( (robotx - goalx) ** 2 + (roboty - goaly)**2   ) < self.thres:
            self.goal += 1
            if self.goal == len(self.goals):
                self.goal = 0

        goal_msg = Odometry()
        goal_msg.pose.pose.position.x = goalx 
        goal_msg.pose.pose.position.y = goaly 


        self.goal_pub.publish(goal_msg)

    

if __name__ == "__main__":

    controller_node = GoalPublisher()
    rospy.spin()





