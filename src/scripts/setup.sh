#!/bin/bash

MACHINE=$(uname -m)

# Set all the required environment variables depending on which machine
# is running the code.
if [[ $MACHINE == "armv7l" ]]; then
  source /home/pi/.bashrc
  RPI_IP=$(hostname -I | awk -F ' ' '{print $1}')

  export ROS_MASTER_URI=http://$RPI_IP:11311
  export ROS_HOSTNAME=$RPI_IP

elif [[ $MACHINE == "x86_64" ]]; then
  RPI_IP=$(ping raspberrypi.local -c 1 | head -n 1 | awk -F'[)(]' '{print $2}')
  HOST_IP=$(hostname -I | awk -F ' ' '{print $1}')

  if [ -z "$RPI_IP" ]; then
    export ROS_MASTER_URI=http://$HOST_IP:11311
    echo "Unable to find RPi on the network. Setting ROS_MASTER_URI to point to self."
  else 
    export ROS_MASTER_URI=http://$RPI_IP:11311

    echo "Found RPi on the network with IP:$RPI_IP. Setting the ROS_MASTER_URI to point to the RPi."
  fi
  export ROS_HOSTNAME=$HOST_IP
fi

# Print the settings to screen
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"

# Setup files for ROS
source /opt/ros/melodic/setup.bash
source devel/setup.bash

## Blacklist the necessary packages
if [[ $MACHINE == "armv7l" ]]; then
  BLACKLIST="stoch3_gazebo;stoch3_description;stoch3_rviz;stoch3_analysis"
  SIM_BUILD="OFF"
elif [[ $MACHINE == "x86_64" ]]; then
  BLACKLIST="" #"stoch3_hardware_interface"
  SIM_BUILD="ON"
fi


## Alias for commands
alias cm="catkin_make -DCATKIN_BLACKLIST_PACKAGES='$BLACKLIST' -DSIM_BUILD=$SIM_BUILD"
if [[ $MACHINE == "armv7l" ]]; then
  alias stop="moteus_tool --pi3hat-cfg '1=11,12,13;2=21,22,23;3=31,32,33;4=41,42,43' -t11-13,21-23,31-33,41-43 --stop"
  alias dcfg="moteus_tool --pi3hat-cfg '1=11,12,13;2=21,22,23;3=31,32,33;4=41,42,43' -t11-13,21-23,31-33,41-43 --dump-config"
  alias dcfg1="moteus_tool --pi3hat-cfg '1=11,12,13' -t11-13 --dump-config"
  alias dcfg2="moteus_tool --pi3hat-cfg '2=21,22,23' -t21-23 --dump-config"
  alias dcfg3="moteus_tool --pi3hat-cfg '3=31,32,33' -t31-33 --dump-config"
  alias dcfg4="moteus_tool --pi3hat-cfg '4=41,42,43' -t41-43 --dump-config"

  alias dcfg11="moteus_tool --pi3hat-cfg '1=11,12,13' -t11 --dump-config"
  alias dcfg12="moteus_tool --pi3hat-cfg '1=11,12,13' -t12 --dump-config"
  alias dcfg13="moteus_tool --pi3hat-cfg '1=11,12,13' -t13 --dump-config"

  alias dcfg21="moteus_tool --pi3hat-cfg '2=21,22,23' -t21 --dump-config"
  alias dcfg22="moteus_tool --pi3hat-cfg '2=21,22,23' -t22 --dump-config"
  alias dcfg23="moteus_tool --pi3hat-cfg '2=21,22,23' -t23 --dump-config"

  alias dcfg31="moteus_tool --pi3hat-cfg '3=31,32,33' -t31 --dump-config"
  alias dcfg32="moteus_tool --pi3hat-cfg '3=31,32,33' -t32 --dump-config"
  alias dcfg33="moteus_tool --pi3hat-cfg '3=31,32,33' -t33 --dump-config"

  alias dcfg41="moteus_tool --pi3hat-cfg '4=41,42,43' -t41 --dump-config"
  alias dcfg42="moteus_tool --pi3hat-cfg '4=41,42,43' -t42 --dump-config"
  alias dcfg43="moteus_tool --pi3hat-cfg '4=41,42,43' -t43 --dump-config"
fi

