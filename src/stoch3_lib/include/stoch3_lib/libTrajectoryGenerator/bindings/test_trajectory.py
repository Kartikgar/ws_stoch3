from build.TrajectoryCore import *

trajCore = TrajectoryCore()

trajCore.initializeRobot()
trajCore.printConfigs()

print("####")

action_test = [0.1]*15
prev_leg_pos_vals_test = [0.57]*3
dt_test = 0.01

gen_traj_test = trajCore.generateTrajectory(action_test, prev_leg_pos_vals_test, dt_test)

trajCore.setWalkingHeight(0.1)
trajCore.setSwingHeight(5.0)
trajCore.setMaxLinearXVel(100)

print(trajCore.getWalkingHeight())
print(trajCore.getSwingHeight())
print(trajCore.getMaxLinearXVel())

trajCore.printConfigs()


print(gen_traj_test)
print("####")
