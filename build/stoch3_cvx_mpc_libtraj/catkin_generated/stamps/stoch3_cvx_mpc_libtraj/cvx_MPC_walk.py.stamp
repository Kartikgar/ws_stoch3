#!/usr/bin/env python3

import rospy

from stoch3_msgs.msg import QuadrupedLegCommand
from stoch3_msgs.msg import QuadrupedRobotState
from sensor_msgs.msg import Joy

import numpy as np
from controller_walk import mpc_controller
from scipy.spatial.transform import Rotation as R
from stoch3_params import *

Xt_global = {}
cmd_global = {}
last_cmd_global = {}
last_cmd_global['vx'] = 0
last_cmd_global['vy'] = 0
last_cmd_global['wz'] = 0

def GetRollPitchYawrate(rpy, ang_vel):
    roll, pitch, yaw = rpy.tolist()
    A = np.array([
    [np.cos(yaw) / np.cos(pitch), np.sin(yaw) / np.cos(pitch), 0],
    [-np.sin(yaw), np.cos(yaw), 0],
    [np.cos(yaw) * np.tan(pitch), np.sin(yaw) * np.tan(pitch), 1]])
    rpy_rate = A @ np.reshape(ang_vel, (-1,1))
    return rpy_rate.reshape(3)

def ConvertBodytoLegFrame(foot_pos):
    leg_frames = np.array([
            [BODY_LENGTH/2.0, BODY_LENGTH/2.0, -BODY_LENGTH/2.0, -BODY_LENGTH/2.0],
            [BODY_WIDTH/2.0, -BODY_WIDTH/2.0, BODY_WIDTH/2.0, -BODY_WIDTH/2.0],
            [0, 0, 0, 0]
        ])
    return foot_pos - leg_frames

def ConvertLegtoBodyFrame(foot_pos):
    leg_frames = np.array([
            [BODY_LENGTH/2.0, BODY_LENGTH/2.0, -BODY_LENGTH/2.0, -BODY_LENGTH/2.0],
            [BODY_WIDTH/2.0, -BODY_WIDTH/2.0, BODY_WIDTH/2.0, -BODY_WIDTH/2.0],
            [0, 0, 0, 0]
        ])
    return foot_pos + leg_frames

'''
Get the required state information from this robotState callback
'''
def robotStateCB(data):

    # position of com in world frame
    position = np.zeros(3)
    position[0] = data.pose.position.x
    position[1] = data.pose.position.y
    position[2] = data.pose.position.z

    # orientation of base in world frame
    orientation = np.zeros(4)
    orientation[0] = data.pose.orientation.x
    orientation[1] = data.pose.orientation.y
    orientation[2] = data.pose.orientation.z
    orientation[3] = data.pose.orientation.w

    # Euler angle representation
    r = R.from_quat([orientation[0], orientation[1], orientation[2], orientation[3]])
    rotation_mat = r.as_matrix()
    yaw, pitch, roll = r.as_euler('zyx', degrees =False)
    euler_angles = [roll, pitch, yaw]

    # Linear velocity of base in world frame
    lin_vel = np.zeros(3)
    lin_vel[0] = data.twist.linear.x
    lin_vel[1] = data.twist.linear.y
    lin_vel[2] = data.twist.linear.z

    # Convert to body-frame
    lin_vel = rotation_mat.T @ np.reshape(lin_vel, (-1,1))

    # angular velocity in world frame
    ang_vel = np.zeros(3)
    ang_vel[0] = data.twist.angular.x
    ang_vel[1] = data.twist.angular.y
    ang_vel[2] = data.twist.angular.z

    # foot-positions in world-frame
    foot_positions = np.zeros(12)
    foot_positions[0] = data.fl.position.x
    foot_positions[1] = data.fl.position.y
    foot_positions[2] = data.fl.position.z

    foot_positions[3] = data.fr.position.x
    foot_positions[4] = data.fr.position.y
    foot_positions[5] = data.fr.position.z

    foot_positions[6] = data.bl.position.x
    foot_positions[7] = data.bl.position.y
    foot_positions[8] = data.bl.position.z

    foot_positions[9] = data.br.position.x
    foot_positions[10] = data.br.position.y
    foot_positions[11] = data.br.position.z

    # Convert to body-frame
    foot_positions = np.reshape(foot_positions, (3, 4), order='F')
    foot_positions = foot_positions - np.reshape(position, (-1, 1))
    foot_positions = rotation_mat @ foot_positions

    # foot_contacts
    prob_thresh = 0.8
    foot_contacts = np.array([data.fl.support_probability > prob_thresh,
                              data.fr.support_probability > prob_thresh,
                              data.bl.support_probability > prob_thresh,
                              data.br.support_probability > prob_thresh])

    global Xt_global

    Xt_global['Rotation_mat'] = rotation_mat
    Xt_global["BaseRollPitchYaw"] = np.array(euler_angles)*0
    Xt_global['BaseRollPitchYawRate'] = GetRollPitchYawrate(rpy=np.array(euler_angles),
                                                            ang_vel=ang_vel)
    Xt_global['FootPositionsInLegFrame'] = ConvertBodytoLegFrame(foot_positions)
    Xt_global['FootPositionsInBaseFrame'] = foot_positions.T
    Xt_global['com_velocity_body_frame'] = lin_vel.reshape(3)
    Xt_global['FootContacts'] = foot_contacts


'''
Look at joy_interface.h, joy_teleop.h from stoch3_teleop for more info
'''
def joyCB(data):
    global cmd_global
    global last_cmd_global
    alpha = 0.8

    cmd_global['vx'] = -0.4 * data.axes[1]
    cmd_global['vy'] = -0.4 * data.axes[3]
    cmd_global['wz'] = -0.4 * data.axes[0]

    cmd_global['vx'] = (1-alpha) * last_cmd_global['vx'] + (alpha) * cmd_global['vx']
    cmd_global['vy'] = (1-alpha) * last_cmd_global['vy'] + (alpha) * cmd_global['vy']
    cmd_global['wz'] = (1-alpha) * last_cmd_global['wz'] + (alpha) * cmd_global['wz']


'''
Put together the necessary information in the format of 
QuadrupedLegCommand message.
'''
def set_info(bool_in_stance, des_foot_force, des_foot_pos):
    
    pure_position_control = False
    
    if pure_position_control:
        kp_scale_stance = 1.0
        kd_scale_stance = 1.0
        kp_scale_swing = 1.0
        kd_scale_swing = 0.5
    else:
        kp_scale_stance = 0.25
        kd_scale_stance = 1.0
        kp_scale_swing = 1.0
        kd_scale_swing = 0.5

    info = QuadrupedLegCommand()
    info.header.frame_id = 'leg'

    info.header.stamp = rospy.Time.now()

    info.fl.position.x, info.fl.position.y, info.fl.position.z = des_foot_pos[0], des_foot_pos[1], des_foot_pos[2]
    info.fr.position.x, info.fr.position.y, info.fr.position.z = des_foot_pos[3], des_foot_pos[4], des_foot_pos[5]
    info.bl.position.x, info.bl.position.y, info.bl.position.z = des_foot_pos[6], des_foot_pos[7], des_foot_pos[8]
    info.br.position.x, info.br.position.y, info.br.position.z = des_foot_pos[9], des_foot_pos[10], des_foot_pos[11]

    if pure_position_control:
        info.fl.force.x, info.fl.force.y, info.fl.force.z = 0.0, 0.0, 0.0
        info.fr.force.x, info.fr.force.y, info.fr.force.z = 0.0, 0.0, 0.0
        info.bl.force.x, info.bl.force.y, info.bl.force.z = 0.0, 0.0, 0.0
        info.br.force.x, info.br.force.y, info.br.force.z = 0.0, 0.0, 0.0
    else:
        info.fl.force.x, info.fl.force.y, info.fl.force.z = des_foot_force[0], des_foot_force[1], des_foot_force[2]
        info.fr.force.x, info.fr.force.y, info.fr.force.z = des_foot_force[3], des_foot_force[4], des_foot_force[5]
        info.bl.force.x, info.bl.force.y, info.bl.force.z = des_foot_force[6], des_foot_force[7], des_foot_force[8]
        info.br.force.x, info.br.force.y, info.br.force.z = des_foot_force[9], des_foot_force[10], des_foot_force[11]

    info.fl.velocity.x, info.fl.velocity.y, info.fl.velocity.z = 0.0, 0.0, 0.0
    info.fr.velocity.x, info.fr.velocity.y, info.fr.velocity.z = 0.0, 0.0, 0.0
    info.bl.velocity.x, info.bl.velocity.y, info.bl.velocity.z = 0.0, 0.0, 0.0
    info.br.velocity.x, info.br.velocity.y, info.br.velocity.z = 0.0, 0.0, 0.0

    info.fl.kp_scale = kp_scale_stance * bool_in_stance[0] + kp_scale_swing * (1.0 - bool_in_stance[0])
    info.fr.kp_scale = kp_scale_stance * bool_in_stance[1] + kp_scale_swing * (1.0 - bool_in_stance[1])
    info.bl.kp_scale = kp_scale_stance * bool_in_stance[2] + kp_scale_swing * (1.0 - bool_in_stance[2])
    info.br.kp_scale = kp_scale_stance * bool_in_stance[3] + kp_scale_swing * (1.0 - bool_in_stance[3])

    info.fl.kd_scale = kd_scale_stance * bool_in_stance[0] + kd_scale_swing * (1.0 - bool_in_stance[0])
    info.fr.kd_scale = kd_scale_stance * bool_in_stance[1] + kd_scale_swing * (1.0 - bool_in_stance[1])
    info.bl.kd_scale = kd_scale_stance * bool_in_stance[2] + kd_scale_swing * (1.0 - bool_in_stance[2])
    info.br.kd_scale = kd_scale_stance * bool_in_stance[3] + kd_scale_swing * (1.0 - bool_in_stance[3])

    return info


'''
Adjust the foot-forces to be w.r.to the body-frame.
'''
def adjust_foot_force(state, des_foot_force):
    R = state['Rotation_mat']

    pf34 = np.reshape(des_foot_force, (3, 4), order='F')
    des_foot_force = np.reshape(np.dot(R.T, pf34), (12, ), order='F')

    return des_foot_force


if __name__ == '__main__':

    rospy.init_node('stoch3_cvx_mpc_libtraj', anonymous=True)

    robot_state_sub = rospy.Subscriber("/stoch3/robot_state", QuadrupedRobotState, robotStateCB, queue_size=1)
    # joy_sub = rospy.Subscriber("/stoch3/joy", Joy, joyCB, queue_size=1)
    leg_cmd_pub = rospy.Publisher('/stoch3/controller/leg_controller/command', QuadrupedLegCommand, queue_size=1)

    i = 0

    rate = rospy.Rate(int(100))

    for counter in range(5):
        rate.sleep()

    init_state = Xt_global
    controller = mpc_controller(init_state=init_state)
    FootContacts = [True, True, True, True]

    while not rospy.is_shutdown():

        state = Xt_global
        state['FootContacts'] = FootContacts
        cmd_vel = cmd_global

        des_GRFs, des_foot_pos, FootContacts = controller.get_action(state, cmd_vel)

        # des_GRFs = adjust_foot_force(state, des_GRFs)
        # bool_in_stance = state['FootContacts']
        bool_in_stance = FootContacts

        # Publish foot forces and positions
        info = set_info(bool_in_stance, des_GRFs, des_foot_pos)
        leg_cmd_pub.publish(info)
        
        i = i + 1

        rate.sleep()
        
    rospy.shutdown()