from build.Serial3RKinematics import *

s3r = Serial3RKinematics()

print("### Test in workspace ###")
ee_pos_test = [1.0]*3
print(s3r.inWorkspace(ee_pos_test))

print("#### Test for forward kinematics ####")
leg_test = "fl"
joint_angles_test = [0.785398]*3
ee_pos_test = [0]*3
ee_pos_test = s3r.forwardKinematics(leg_test, joint_angles_test)
print(ee_pos_test)

print("#### Test for inverse kinematics ####")
inv_joint_angle_test = [0]*3
branch_test = '<'
valid_test = s3r.inverseKinematics(leg_test, ee_pos_test, branch_test, inv_joint_angle_test)
if(valid_test>=0):
    print("Valid ik: ", valid_test)
    inv_joint_angle_test = s3r.inverseKinematics(leg_test, ee_pos_test, branch_test)
    print(inv_joint_angle_test)
else:
    print("Invalid ik")
