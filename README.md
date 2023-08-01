# ws_stoch3
### Description 
This repo contains the workspace for the perception stack of stoch3. Currently this workspace supports elevation mapping, octomapping, and calculating surface normals on stoch3 robot.
### Installation Instruction 

1) Git clone the repo
2) Run: <br> `cd ~/ws_stoch3` <br>
`rosdep install --from-paths src --ignore-src -r -y`  
3) Run: `catkin_make`

### For mapping

1) To install point cloud to laser package run <br>
`sudo apt-get install ros-melodic-pointcloud-to-laserscan ros-$distro-rosbridge-server`
2) To install gmapping dependencies run  <br>
`sudo apt-get install ros-$distro-openslam-gmapping`

### Running the code
1) To run the elevation mapping/ocotmapping/surface normals run <br> `roslaunch stoch3 stoch3_mapping.launch` 
2) To set initial configuration, in a separate terminal run <br> `rosrun stoch3_gazebo set_model_configuration_node`

### Additional notes
1) Start the motors with `V`, sitting position with `R`, standing with `Y`, waling with `H`, use `W A D X` for controlling the motion
2) Run `rosrun rviz rviz` to visualize the different maps and surface normals

