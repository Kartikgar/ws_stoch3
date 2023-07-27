# Calibration
This document explains the procedure to calibrate the motors of the robot, i.e., set its zero-offsets, so as to match the conventions that we follow. 

## Conventions
The convention that we use are described in the subsections that follow:

#### Reference frame
The reference frame is oriented in a way that the X-axis points forward, and Z-axis points upward. The Y-axis can then be determined using the right hand thumb rule. In this case the Y-axis points to the left side of the robot.

#### Measurement of angles
The angles are measured such the Counter-Clockwise rotation is considrered positive while Clockwise rotation is considered negative. The abduction joints rotate about the positive X-axis. The hip and knee joints rotate about the positive Y-axis (in the plane of the leg, where the Y-axis points to the left). All the joints irrespective of whether the are on the front/back legs or left/right legs follow the same convention.

#### Zero angle reference
The abduction angle is considered zero when the hip/knee motors are perfectly horizontal (or their axes align with the body Y-axis ).
The hip and knee angles are considered to be zero when the leg is vertically extended in the X-Z plane.  


## Calibration procedure
While assembling the motors, it is not always possible to ensure that the zero angles of the motors align with the zero angles according to our convention. A zero offset is configured in software to convert between the actual motor angles and the joint angles according to our convention. The zero-offsets are stored in a configuration file `stoch3_hardware_interface/config/motor_offsets.yaml`.  

The zero angles of the motors can also be configured using the `moteus_tool` software. It is preferred to set the motor zero position at a known pose of the robot. The robot is usually placed on the ground (with all legs folded) before it is powered on. Hence, it may be useful to set this pose as the zero so that there is no ambiguity in the motors positions (which can be caused due to the use of a transmission and only a motor side encoder) when the robot is powered up. To set the motor zero-offset use the following command:
` moteus_tool --pi3hat-cfg '<x>=<nn>' -t <nn> --zero-offset`
where `<x>` is the bus number (which can be 1, 2, 3, or 4) and `<nn>` is the motor id (which can be 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, or 43).
The above command must be used for all the 12 actuators (11 through 43). After the zero is set, the motor offsets can be determined using the procedure detailed below. Make sure that the motors are still on and the robot moved on to a stand so that the joints can be moved without any obstruction.

The procedure to be followed to set the calibration is:
1) Set the offsets in the configuration file to zero.
2) Set the robot in on the ground with all legs folded
3) Power up the robot (the actuators)
4) Launch the code in the RPi using `roslaunch stoch3 hardware.launch enable_motors:=false`.
5) Launch the node `rosrun stoch3_hardware_interface stoch3_joint_offset_calibration_node`
6) Move all the joints slowly from one limit to the other
7) Once all the joints are moved, from another terminal make the service call `rosservice call /stoch3_joint_offset_calibration/print_offset_calibration`
8) The motor offsets will be printed in the terminal that is running the `stoch3_joint_offset_calibration_node` 
9) Update the calculated zero-offset values to the `motor_offsets.yaml` configuration file. 
10) Verify that the offsets are configured correctly either by reading the values from `/stoch3/joint_states` and ensuring they match with the expected joint angles, or use the RViz visualization to compare the actual configuration of the robot to the displayed configuration. 

## Theory

The joint angle (following the above mentioned convention) is determined from the motor angle (value read from the motor) using the following equation:
```
joint_angle = direction*motor_angle - zero_offset
```

When we initially set the zero_offset to 0 and read the angle we get:
```
read_value = direction*motor_angle
```

So in simple terms:
```
expected_value = read_value - zero_offset
```
or
```
zero_offset = read_value - expected_value
```

Note: See the implementation of functions ` convert_jointcommand_to_motorcommand` and `convert_motorangle_to_jointangle` in the `stoch3_hardware_interface_node`.
