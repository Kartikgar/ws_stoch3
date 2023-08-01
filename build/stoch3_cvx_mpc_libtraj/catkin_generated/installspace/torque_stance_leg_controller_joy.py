# Lint as: python3
"""A torque based stance controller framework."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
from typing import Any, Sequence, Tuple
import numpy as np

try:
  import leg_controller
  import mpc_osqp as convex_mpc
except:  # pylint: disable=W0702
  print("You need to install motion_imitation")
  print("Either run python3 setup.py install --user in this repo")
  print("or use pip3 install motion_imitation --user")
  sys.exit()


# The QP weights in the convex MPC formulation. See the MIT paper for details:
#   https://ieeexplore.ieee.org/document/8594448/
# Intuitively, this is the weights of each state dimension when tracking a
# desired CoM trajectory. The full CoM state is represented by
# (roll_pitch_yaw, position, angular_velocity, velocity, gravity_place_holder).
# _MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0.5, 0.5, 0.2, 0.2, 0.2, 0.1, 0)
# This worked well for in-place stepping in the real robot.
# _MPC_WEIGHTS = (5, 5, 0.2, 0, 0, 10, 0., 0., 0.2, 1., 1., 0., 0)

_MPC_WEIGHTS = (5, 5, 10, 0, 0, 10, 0., 0., 0., 1., 1., 0., 0)
_PLANNING_HORIZON_STEPS = 10
_PLANNING_TIMESTEP = 0.025

class TorqueStanceLegController(leg_controller.LegController):
  """A torque based stance leg controller framework.

  Takes in high level parameters like walking speed and turning speed, and
  generates necessary the torques for stance legs.
  """
  def __init__(
      self,
      desired_speed: Tuple[float, float] = (0, 0),
      desired_twisting_speed: float = 0,
      desired_body_height: float = 0.45,
      body_mass: float = 220 / 9.8,
      body_inertia: Tuple[float, float, float, float, float, float, float,
                          float, float] = (0.07335, 0, 0, 0, 0.25068, 0, 0, 0,
                                           0.25447),
      num_legs: int = 4,
      friction_coeffs: Sequence[float] = (0.45, 0.45, 0.45, 0.45),
      qp_solver = convex_mpc.QPOASES,
      mpc_method = 'cvx',
      A_mat=np.zeros((13*13),dtype=np.float32).reshape(13,13).tolist(), 
      B_mat=np.zeros((13*12),dtype=np.float32).reshape(13,12).tolist()
  ):
    """Initializes the class.

    Tracks the desired position/velocity of the robot by computing proper joint
    torques using MPC module.

    Args:
      robot: A robot instance.
      state_estimator: Estimate the robot states (e.g. CoM velocity).
      desired_speed: desired CoM speed in x-y plane.
      desired_twisting_speed: desired CoM rotating speed in z direction.
      desired_body_height: The standing height of the robot.
      body_mass: The total mass of the robot.
      body_inertia: The inertia matrix in the body principle frame. We assume
        the body principle coordinate frame has x-forward and z-up.
      num_legs: The number of legs used for force planning.
      friction_coeffs: The friction coeffs on the contact surfaces.
    """
    self.desired_speed = desired_speed
    self.desired_twisting_speed = desired_twisting_speed

    self._desired_body_height = desired_body_height
    self._body_mass = body_mass
    self._num_legs = num_legs
    self._friction_coeffs = np.array(friction_coeffs)
    body_inertia_list = list(body_inertia)
    weights_list = list(_MPC_WEIGHTS)
    self._cpp_mpc = convex_mpc.ConvexMpc(
        body_mass,
        body_inertia_list,
        self._num_legs,
        _PLANNING_HORIZON_STEPS,
        _PLANNING_TIMESTEP,
        weights_list,
        1e-5,
        qp_solver   
    )
    self._mpc_method = mpc_method
    self.Ut = np.zeros((self._num_legs*3, 1), dtype=np.float64)
    self.Ut[[2,5,8,11], :] = np.array([self._body_mass*9.8/4 for _ in range(self._num_legs)]).reshape(-1, 1)
    self._step_mpc = 0
    self._cvx_freq = 1
    self.Ud = np.zeros((12, _PLANNING_HORIZON_STEPS), dtype=np.float64)


  def set_A_B_mat(self, A_mat, B_mat, ch=0):
    self._cpp_mpc.setAB(A_mat, B_mat, int(ch))

  def reset(self, current_time):
    del current_time

  def update(self, current_time):
    del current_time

  def get_action(self, 
                 BaseRollPitchYaw, 
                 BaseRollPitchYawRate,
                 FootPositionsInBaseFrame,
                 FootContacts,
                 com_velocity_body_frame) -> Any:
    if self._mpc_method == 'cvx':
      return self.get_action_cvx(BaseRollPitchYaw,
                                 BaseRollPitchYawRate,
                                 FootPositionsInBaseFrame,
                                 FootContacts,
                                 com_velocity_body_frame)
    else:
      print('ERROR: MPC Method invalid')
      raise NotImplementedError

  def get_action_cvx(self, 
                     BaseRollPitchYaw, 
                     BaseRollPitchYawRate,
                     FootPositionsInBaseFrame,
                     FootContacts,
                     com_velocity_body_frame):
    """Computes the torque for stance legs."""

    desired_com_position = np.array((0., 0., self._desired_body_height),
                                    dtype=np.float64)
    desired_com_velocity = np.array(
        (self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
    desired_com_roll_pitch_yaw = np.array((0., 0., 0.), dtype=np.float64)
    desired_com_angular_velocity = np.array(
        (0., 0., self.desired_twisting_speed), dtype=np.float64)

    foot_contact_state = FootContacts

    # We use the body yaw aligned world frame for MPC computation.
    com_roll_pitch_yaw = np.array(BaseRollPitchYaw, dtype=np.float64)
    com_roll_pitch_yaw[2] = 0
  
    if self._step_mpc % self._cvx_freq == 0:
      predicted_contact_forces = self._cpp_mpc.compute_contact_forces([0],  #com_position     #np.asarray(self._state_estimator.com_velocity_body_frame,                    
                                                                      np.asarray(com_velocity_body_frame,dtype=np.float64),  #com_velocity
                                                                      np.array(com_roll_pitch_yaw, dtype=np.float64),  #com_roll_pitch_yaw
                                                                      # Angular velocity in the yaw aligned world frame is actually different
                                                                      # from rpy rate. We use it here as a simple approximation.
                                                                      np.asarray(BaseRollPitchYawRate, dtype=np.float64),  #com_angular_velocity
                                                                      foot_contact_state,  #foot_contact_states
                                                                      np.array(FootPositionsInBaseFrame.flatten(),dtype=np.float64),  #foot_positions_base_frame
                                                                      self._friction_coeffs,  #foot_friction_coeffs
                                                                      desired_com_position,  #desired_com_position
                                                                      desired_com_velocity,  #desired_com_velocity
                                                                      desired_com_roll_pitch_yaw,  #desired_com_roll_pitch_yaw
                                                                      desired_com_angular_velocity  #desired_com_angular_velocity
                                                                    )
      self.Ud = np.array(predicted_contact_forces).reshape((12, _PLANNING_HORIZON_STEPS), order='F')

    self.Ut = self.Ud[:,0].reshape((-1,1), order='F').astype(np.float64)
    predicted_contact_forces = (self.Ut).flatten(order='F')
    
    return predicted_contact_forces

