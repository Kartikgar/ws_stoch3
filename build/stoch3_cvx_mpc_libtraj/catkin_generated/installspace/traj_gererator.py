import stoch
import numpy as np
from stoch3_params import *

SWING_HEIGHT = 0.10
STANCE_HEIGHT = 0.0
STEP_FREQUENCY = 2.0
MAX_VEL_LINEAR_X = 1.5
MAX_VEL_LINEAR_Y = 1.0
MAX_VEL_LINEAR_Z = 0.2
MAX_VEL_ANGULAR_X = 0.5
MAX_VEL_ANGULAR_Y = 0.5
MAX_VEL_ANGULAR_Z = 1.5
M_PI = np.pi

# Vertical Trajectory Type
SINE=0,
CSPLINE1=1,
CSPLINE2=2

class TrajGenerator:
    def __init__(self, gait) -> None:
        self.traj_gen = stoch.TrajectoryCore(True)
        self.traj_robot_data = stoch.RobotData()
        self.traj_gait = stoch.Gait()
        self.gait = gait

    def reset(self, foot_pos) -> None:
        # set gait parameters
        self.traj_gait.setType("trot")
        self.traj_gait.setFreq(STEP_FREQUENCY)
        self.traj_gait.setPhase([0, M_PI, M_PI, 0])
        # self.traj_gait.setStanceDuration([
        #     M_PI + (M_PI/5.),
        #     M_PI + (M_PI/5.),
        #     M_PI + (M_PI/5.),
        #     M_PI + (M_PI/5.)])  # Trot gait
        self.traj_gait.setWalkingHeight(-0.45)
        self.traj_gait.setSwingHeight(SWING_HEIGHT)
        self.traj_gait.setStanceHeight(STANCE_HEIGHT)
        self.traj_gait.setMaxLinearVel(
            [MAX_VEL_LINEAR_X, MAX_VEL_LINEAR_Y, MAX_VEL_LINEAR_Z])
        self.traj_gait.setMaxAngularVel(
            [MAX_VEL_ANGULAR_X, MAX_VEL_ANGULAR_Y, MAX_VEL_ANGULAR_Z])
        
        # set robot data
        link_lengths = [ABD_LEN, THIGH_LEN, SHANK_LEN]  # <ABD, THIGH, SHANK>
        robot_body_dim = [BODY_LENGTH, BODY_WIDTH]  # <Length, Width>

        self.traj_robot_data.setRobotDimensions(link_lengths, robot_body_dim)
        self.traj_robot_data.setRobotLinkLengths(link_lengths)
        self.traj_robot_data.initializeFootPos(foot_pos)

        self.traj_gen.setGaitConfig(self.traj_gait)
        self.traj_gen.setRobotData(self.traj_robot_data)
        self.traj_gen.resetTheta()