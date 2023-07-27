## IMU Reference Orientaion Setting:

To set the reference frame of the IMU so that it aligns with the reference frame of the robot body set the RotSensor matrix in the IMU using the MT Manager software. The process to follow is as follows:
 - Connect the sensor to a computer running the `MT Manager` software.
 - Ensure that the `MT Manager` detects the sensor.
 - Goto : `Device Settings > Device Settings`
 - Update the `RotSensor` rotation matrix.

Refer to [this](https://base.xsens.com/s/article/Changing-or-Resetting-the-MTi-reference-co-ordinate-systems-1605869706643?language=en_US) document for more details.

Alternatively, the following method can be used to set the reference orientation:
1) Align the robot in a position that is considered to have zero roll and zero pitch (at least close to zero roll and zero pitch)
2) Use the `inclination reset` functionality in the `MT Manager` software to set the `RotSensor` matrix. 
3) Save the new inclination by clicking on the save button.
4) Make changes in the `RotSensor` matrix by changing all values that are close to 0 to 0 and all values that are close to 1 to 1 and values close to -1 to -1.

Note: This method works on the assumption that the new orientation of the sensor with respect to the body has roll and pitch that are multiples of 90 degrees.
