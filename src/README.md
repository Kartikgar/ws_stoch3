# Stoch3 ROS packages

This repository contains the packages used to simulate and control the Stoch3 robot.

## Convention

The convention for assigning reference frames is:

Forward is +ve X
Left is +ve Y
Upward is +ve Z

## Installing ros with docker

Instructions for installing ros within docker can be found [here](docs/ros_docker_install.md).

## Usage

Create a workspace for stoch3 and clone the repositories
```
$ mkdir -p ws_stoch3/src
$ cd ws_stoch3/src
$ git clone --recursive <repository> ./
```

Setup the environment
```
$ cd ws_stoch3
$ source src/scripts/setup.sh
```

Compile the applications:
```
$ cm
```
Note: `cm` is an alias that is set for the `catkin_make` call in the `src/scripts/setup.sh`. It also provides the list of packages
to be blacklisted based on the machine in which the code is being built.

Note: This repository in meant to used both on the RPi and the regular x86_64 machines. While the RPi will not support
simulation related packages (Gazebo, RViz etc.), the x86_64 machine will not support the hardware interface
packages. This will lead to build failures if all the packages are compiled on any machine. The `setup.sh` script will
blacklist the relevant packages based on which machine the code is being compiled on. Hence, the script must be run atleast
once when the repository is newly created. The list of packages to be blacklisted will be stored in the CMake cache files. Once this is done the normal build command `catkin_make` can be used to compile the code for future builds.


### Robot
To run the applications on the robot launch the hardware interface (and all other controllers)
```
$ roslaunch stoch3 hardware.launch enable_motors:=true
```
This will launch all the required nodes and put the robot in an `idle` state. The parameter `enable_motors` is set to false by default. Hence it has to be explicitly set to `true` to enable the motors. To disable the motors, set the parameter `enable_motors:=false` during launch.

### Simulation
  
Launch the Gazebo simulation of the robot:
```
$ roslaunch stoch3 gazebo_sim.launch
```
To set the robot in a zero position, run the following command
in another terminal:
```
$ rosrun stoch3_gazebo set_model_configuration_node
```
Note: This node needs to be run before sending any commands to the robot from the teleop interface.

### Visualization

To visualize the robot in RViz, use the following launch file:
```
$ roslaunch stoch3 rviz.launch
```
This launch file can be used either when the real robot is being controlled or when a simulation environment is being used, i.e., use the `rviz.launch` file in conjunction with `hardware.launch` or `gazebo_sim.launch`. However, if you need to use RViz independently, then use the following launch file:
```
$ roslaunch stoch3_description stoch3_rviz.launch
```

Note: If you wish to use RViz in conjunction with the robot, then do not forget to `source src/scripts/setup.sh` before launching the RViz applications. This script will set all the required environment variables to allow ROS to detect the ros master that is being run remotely (on the robot).

## Operation

The robot uses a state-machine to transition between different states that provide different functionalities. The details
of the state-machine can be seen [here](docs/state_machine.md)

## Remote Access

##### Note:
All the required environment variables will be set when the `src/scripts/setup.sh` is sourced.

To communicate over ROS to a remote computer follow the instructions below.

On the robot computer:
```
export ROS_MASTER_URI=http://<ROBOT_COMPUTER_IP>:11311
export ROS_HOSTNAME=<ROBOT_COMPUTER_IP>
```
These two environment variables have to be set in every terminal. To make it simple set the values in the `.bashrc` file:
```
echo 'export ROS_MASTER_URI=http://<ROBOT_COMPUTER_IP>:11311' >> ~/.bashrc
echo 'export ROS_HOSTNAME=<ROBOT_COMPUTER_IP>' >> ~/.bashrc
```

On the host computer (remote computer running the visualization):
```
export ROS_MASTER_URI=http://<ROBOT_COMPUTER_IP>:11311
export ROS_HOSTNAME=<HOST_COMPUTER_IP>
```

The steps for the setup can be obtained from [this](https://wiki.ros.org/Robots/TurtleBot/Network%20Setup) reference.


## Coding Conventions
The following conventions should be followed for naming the packages and other ROS related entities:
[ROS C++ coding conventions](http://wiki.ros.org/CppStyleGuide)
