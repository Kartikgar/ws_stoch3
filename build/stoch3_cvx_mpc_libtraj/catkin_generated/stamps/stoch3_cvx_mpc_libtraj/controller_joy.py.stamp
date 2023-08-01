from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import scipy.interpolate

import gait_generator as gait_generator_lib
import locomotion_controller_joy as locomotion_controller
import libtraj_swing_leg_controller
import torque_stance_leg_controller_joy as torque_stance_leg_controller

class mpc_controller():
  def __init__(self, init_state) -> None:
    self.MPC_BODY_MASS = 25
    self.MPC_BODY_INERTIA = np.array((0.24, 0, 0, 0, 0.56, 0, 0, 0, 0.89))
    self.MPC_BODY_HEIGHT = 0.45
    
    self.ADD_INIT_WEIGHT = False

    controller = self._setup_controller(self.MPC_BODY_MASS,
                                        self.MPC_BODY_INERTIA,
                                        self.MPC_BODY_HEIGHT)
    
    controller.set_state(init_state)
    controller.reset()
    self.current_time = 0
    self.temp_iterator = 0
    self.controller = controller

    self.last_lin_spped = [0,0,0]
    self.last_ang_speed = 0

  def get_action(self, state, cmd_vel):
    lin_speed = [cmd_vel['vx'], cmd_vel['vy'], 0]
    ang_speed = cmd_vel['wz']
    # lin_speed, ang_speed = self._generate_example_linear_angular_speed(self.current_time)

    # alpha = 0.7
    # for i in range(3):  
    #   lin_speed[i] = (1-alpha)*self.last_lin_spped[i] + (alpha)*lin_speed[i]
    # ang_speed = (1-alpha)*self.last_ang_speed + (alpha)*ang_speed 
    
    self._update_controller_params(lin_speed, ang_speed)

    self.controller.set_state(state)
    self.controller.update()

    contact_forces, foot_positions, FootContacts = self.controller.get_action(state)

    # Increment the step_counter in controller
    self.controller.step_count  
    self.current_time = self.controller.get_clock_tick()
    self.temp_iterator += 1
    
    return contact_forces, foot_positions, FootContacts
  
  @property
  def get_time_since_reset(self):
    self.controller.get_time_since_reset

  def _update_controller_params(self, lin_speed, ang_speed):
    self.controller.swing_leg_controller.desired_speed = lin_speed
    self.controller.swing_leg_controller.desired_twisting_speed = ang_speed
    self.controller.stance_leg_controller.desired_speed = lin_speed
    self.controller.stance_leg_controller.desired_twisting_speed = ang_speed
      
  def _setup_controller(self,
                        MPC_BODY_MASS,
                        MPC_BODY_INERTIA,
                        MPC_BODY_HEIGHT,
                        desired_speed=(0, 0),
                        desired_twisting_speed=0,
                        do_update=False):

    sw_controller = libtraj_swing_leg_controller.LibTrajSwingLegController(
       gait='trot',
       desired_speed=desired_speed,
       desired_twisting_speed=desired_twisting_speed,
       desired_height=MPC_BODY_HEIGHT)

    st_controller = torque_stance_leg_controller.TorqueStanceLegController(
        desired_speed=desired_speed,
        desired_twisting_speed=desired_twisting_speed,
        desired_body_height=MPC_BODY_HEIGHT,
        body_mass=MPC_BODY_MASS,
        body_inertia=MPC_BODY_INERTIA,
        A_mat=np.zeros((13*13),dtype=np.float32).reshape(13,13).tolist(),
        B_mat=np.zeros((13*12),dtype=np.float32).reshape(13,12).tolist())

    controller = locomotion_controller.LocomotionController(
        swing_leg_controller=sw_controller,
        stance_leg_controller=st_controller)

    return controller
  
  def _generate_example_linear_angular_speed(self,t):
    """Creates an example speed profile based on time for demo purpose."""
    

    vx = 0.2
    vy = 0.2
    wz = 0.2

    time_points = (0, 5, 10, 15, 20, 25,30)
    speed_points = ((0, 0, 0, 0),(vx, 0, 0, 0), (-vx, 0, 0, 0), (0, vy, 0, 0), (0, -vy, 0, 0), (0, 0, 0, wz),(0, 0, 0, -wz))
    # speed_points = ((0, 0, 0, 0),(vx, 0, 0, 0), (vx, 0, 0, 0), (vx, 0, 0, 0), (vx, 0, 0, 0), (vx, 0, 0, 0),(vx, 0, 0, 0))

    speed = scipy.interpolate.interp1d(
        time_points,
        speed_points,
        kind="previous",
        fill_value="extrapolate",
        axis=0)(
            t)

    return speed[0:3], speed[3]
