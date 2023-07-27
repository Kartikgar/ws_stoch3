#! /bin/bash

mode_op="key"

if [[ -e /dev/input/js0 ]]; then
    mode_op="joy"
fi
roslaunch stoch3_teleop teleop.launch mode:=$mode_op