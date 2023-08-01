"""The swing leg controller class."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from typing import Any, Mapping

import leg_controller
import traj_gererator
from stoch3_params import *

dt = 0.01

class LibTrajSwingLegController(leg_controller.LegController):
    def __init__(self, gait,
                 desired_speed,
                 desired_twisting_speed,
                 desired_height) -> None:
        super().__init__()
        self.desired_speed = np.array((desired_speed[0], desired_speed[1], 0))
        self.desired_twisting_speed = desired_twisting_speed
        self.desired_height = desired_height
        self.TrajGenerator = traj_gererator.TrajGenerator(gait=gait)

    def reset(self, FootPositionsInLegFrame) -> None:
        self.TrajGenerator.reset(FootPositionsInLegFrame)

    def update(self, current_time) -> None:
        del current_time

    def get_action(self, 
                   FootPositionsInLegFrame,
                   FootContacts) -> Mapping[Any, Any]:
        # shifts = np.array([[0.0, ABD_LEN + 0.0, 0.0],
        #                    [0.0, -ABD_LEN - 0.0, 0.0],
        #                    [0.0, ABD_LEN + 0.0, 0.0],
        #                    [0.0, -ABD_LEN - 0.0, 0.0]], dtype=np.float32)
        shifts = np.array([[0.0, ABD_LEN - 0.0, 0.0],
                           [0.0, -ABD_LEN + 0.0, 0.0],
                           [0.0, ABD_LEN - 0.0, 0.0],
                           [0.0, -ABD_LEN + 0.0, 0.0]], dtype=np.float32)
        shifts = np.transpose(shifts)
        des_lin_vel = self.desired_speed
        des_ang_vel = np.array([0, 0, self.desired_twisting_speed])
        des_foot_pos = self.TrajGenerator.traj_gen.generateTrajectory(shifts, 
                                                                      FootPositionsInLegFrame,
                                                                      FootContacts, des_lin_vel,
                                                                      des_ang_vel, dt)
        
        FootContacts = []
        for leg_name in ["fl", "fr", "bl", "br"]:
            FootContacts.append(self.TrajGenerator.traj_gen.isStanceLeg(leg_name))

        return des_foot_pos, FootContacts