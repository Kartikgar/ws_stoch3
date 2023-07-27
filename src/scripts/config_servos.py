#!/usr/bin/env python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Set servo-level configuration for a quad A1 robot.'''


import os
import subprocess
import sys
import tempfile


SCRIPT_PATH = os.path.dirname(__file__)
MOTEUS_TOOL = ['moteus_tool',
               '--pi3hat-cfg',
               '1=11,12,13;2=21,22,23;3=31,32,33;4=41,42,43',
              # '1=11,12,13;2=21,22,23',
              # '1=21,22,23',
               ]

# position_min and position_max should be changed if the actuator
# is recalibrated. Once the actuator zero is set, read the positions
# of the mechanical limits and use those values to determine the 
# position_min and position_max. Set the soft limits lower than the
# hard mechanical limits (except for the knees). For the knees set
# a lower limit slightly more than the mechanical limits so as not
# to put the motor outside
CONFIG = {
    'servopos.position_min' : [
        ('11', '-0.056'), # -0.056 is mechanical limit
        ('12', '-0.101'), # -0.111 is mechanical limit
        ('13', '-0.439'), # -0.449 is mechanical limit

        ('21', '-0.134'), # -0.130 is mechanical limit
        ('22', '-0.337'), # -0.347 is mechanical limit
        ('23', '-0.006'), #  0.004 is mechanical limit

        ('31', '-0.138'), # -0.130 is mechanical limit
        ('32', '-0.054'), # -0.064 is mechanical limit
        ('33', '-0.439'), # -0.449 is mechanical limit

        ('41', '-0.080'), # -0.064 is mechanical limit
        ('42', '-0.376'), # -0.386 is mechanical limit
        ('43', '-0.008'), #  0.002 is mechanical limit
    ],
    'servopos.position_max' : [
        ('11',  '0.141'), #  0.141 is mechanical limit
        ('12',  '0.329'), #  0.339 is mechanical limit
        ('13', ' 0.015'), # -0.005 is mechanical limit

        ('21',  '0.070'), #  0.066 is mechanical limit
        ('22',  '0.092'), #  0.102 is mechanical limit
        ('23',  '0.451'), #  0.461 is mechanical limit

        ('31',  '0.073'), #  0.065 is mechanical limit
        ('32',  '0.375'), #  0.385 is mechanical limit
        ('33', ' 0.007'), # -0.003 is mechanical limit

        ('41',  '0.146'), #  0.130 is mechanical limit
        ('42',  '0.056'), #  0.066 is mechanical limit
        ('43',  '0.455'), #  0.465 is mechanical limit
    ],
    'servo.pwm_min' : '0.006',
    'servo.flux_brake_min_voltage' : '27.0',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.ilimit' : '0', #5
    'servo.pid_position.ki' : '0.0', #20
    'servo.pid_position.kp' : [
        ('11,12,21,22,31,32,41,42', '200'),
        ('13,23,33,43', '200'),
    ],
    'servo.max_velocity' : '1',
    #'servo.feedforward_scale' : '0.0',
    'servo.pid_position.kd' : '10',
    'servo.pid_dq.ki' : '27',#'150.0',
    'motor.unwrapped_position_scale' : [
        ('11,12,21,22,31,32,41,42', '0.16666667'),
        ('13,23,33,43', '0.12222222'),
    ],

    'servo.derate_temperature' : '70',

}


def run(*args, **kwargs):
    print('RUN: ', *args, **kwargs)
    subprocess.check_call(*args, **kwargs)


def main():
    if os.geteuid() != 0:
        raise RuntimeError('This must be run as root')

    for key, data_or_value in CONFIG.items():
        if type(data_or_value) == str:
            print('It is coming here, data is a string now')
            print('Configuring',key,'with values',data_or_value)
            with tempfile.NamedTemporaryFile(delete=False) as config:
                value = data_or_value

                config.write(
                    'conf set {} {}\n'.format(key, value).encode('utf8'))
                config.flush()

                run(MOTEUS_TOOL + ['-t11,12,13,21,22,23,31,32,33,41,42,43', '--write-config', config.name])
        else:
            data = data_or_value
            for servo_selector, value in data:
                print('It is coming here. Hence the data_or_value type is not string')
                print('Configuring',key,'with value',value)
                with tempfile.NamedTemporaryFile(delete=False) as config:
                    config.write(
                        'conf set {} {}\n'.format(key, value).encode('utf8'))
                    config.flush()
                    run(MOTEUS_TOOL + ['-t{}'.format(servo_selector),
                         '--write-config', config.name])

    # Now store them all persistently.
    with tempfile.NamedTemporaryFile(delete=False) as config:
        config.write(b'conf write\n')
        config.flush()
        run(MOTEUS_TOOL + ['-t11,12,13,21,22,23,31,32,33,41,42,43', '--write-config', config.name])


if __name__ == '__main__':
    main()
