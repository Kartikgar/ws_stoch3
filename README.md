# ws_stoch3
Description
This repo contains the workspace for the perception stack of stoch3. Currently this workspace supports elevation mapping, octomapping, and calculating surface normals on stoch3 robot.

Installation Instruction
Git clone the repo
Run:
cd ~/ws_stoch3
rosdep install --from-paths src --ignore-src -r -y
Run: catkin_make
For mapping
To install point cloud to laser package run
sudo apt-get install ros-melodic-pointcloud-to-laserscan ros-$distro-rosbridge-server
To install gmapping dependencies run
sudo apt-get install ros-$distro-openslam-gmapping
Running the code
To run the elevation mapping/ocotmapping/surface normals run
roslaunch stoch3 stoch3_mapping.launch
To set initial configuration, in a separate terminal run
rosrun stoch3_gazebo set_model_configuration_node
Additional notes
Start the motors with V, sitting position with R, standing with Y, waling with H, use W A D X for controlling the motion
Run rosrun rviz rviz to visualize the different maps and surface normals
