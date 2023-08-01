"""A model based controller framework."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
from typing import Any, Callable


class LocomotionController(object):
  """Generates the quadruped locomotion.

  The actual effect of this controller depends on the composition of each
  individual subcomponent.

  """
  def __init__(self,
               swing_leg_controller,
               stance_leg_controller):
    """Initializes the class.

    Args:
      robot: A robot instance.
      state_estimator: Estimates the state of the robot (e.g. center of mass
        position or velocity that may not be observable from sensors).
      swing_leg_controller: Generates motor actions for swing legs.
      stance_leg_controller: Generates motor actions for stance legs.
      clock: A real or fake clock source.
    """
    
    self.state = None # the state of the robot
    self._step_count = 0
    self._simulation_time_step = 0.01
    self.clock_multiplier = 1   # Not sure what the three is for, but it is needed to work
    self._reset_time = self.get_clock_tick()

    self._time_since_reset = 0
    self._swing_leg_controller = swing_leg_controller
    self._stance_leg_controller = stance_leg_controller
    self._joint_angles = None

    self.L = 0.541  # body length
    self.W = 0.203  # body width
    self.d = 0.123  # ABAD offset
    
    self.hip_positions=(
     (self.L/2, self.W/2+self.d,0),
     (self.L/2, -self.W/2-self.d,0),
     (-self.L/2, self.W/2+self.d,0),
     (-self.L/2, -self.W/2-self.d,0))
    
    self._no_of_motors = 12

  @property
  def swing_leg_controller(self):
    return self._swing_leg_controller

  @property
  def stance_leg_controller(self):
    return self._stance_leg_controller
  
  @property
  def step_count(self):
    self._step_count += 1
  
  @property
  def get_time_since_reset(self):
    return self._time_since_reset
  
  def get_clock_tick(self):
    return self._step_count * self._simulation_time_step * self.clock_multiplier
  
  def set_state(self, state):
    self.state = state

  def set_st_controller(self, st_controller):
    self._stance_leg_controller = st_controller
  
  def update_time_since_reset(self):
    return self.get_clock_tick() - self._reset_time

  def reset(self):
    self._reset_time = self.get_clock_tick()  
    self._time_since_reset = 0
    self._swing_leg_controller.reset(self.state['FootPositionsInLegFrame'])
    self._stance_leg_controller.reset(self._time_since_reset)
    self._joint_angles = {}

  def update(self):
    self._time_since_reset = self.update_time_since_reset()
    self._swing_leg_controller.update(self._time_since_reset)
    self._stance_leg_controller.update(self._time_since_reset)

  def get_action(self, state):
    """Returns the control ouputs (e.g. positions/torques) for all motors."""
    
    self.state = state

    com_velocity_body_frame = state['com_velocity_body_frame']
    BaseRollPitchYawRate = state['BaseRollPitchYawRate']

    BaseRollPitchYaw = state['BaseRollPitchYaw']
    FootPositionsInLegFrame = state['FootPositionsInLegFrame']
    FootPositionsInBaseFrame = state['FootPositionsInBaseFrame']
    FootContacts = state['FootContacts']
    
    foot_positions, FootContacts  = self._swing_leg_controller.get_action(FootPositionsInLegFrame,
                                                           FootContacts)
    
    foot_positions = foot_positions.flatten(order='F')
    
    contact_forces = self._stance_leg_controller.get_action(BaseRollPitchYaw,
                                                            BaseRollPitchYawRate,
                                                            FootPositionsInBaseFrame,
                                                            FootContacts,
                                                            com_velocity_body_frame)

    return contact_forces, foot_positions , FootContacts