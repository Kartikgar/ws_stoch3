#include "utils/matrix_op.h"

utils::Matrix<double, 6, 3> getTofCoordinates()
{
  // old cordinates
  utils::Matrix<double, 6, 3> tof_coordinates;
  tof_coordinates << -0.065, 0.047, 0, //tof1
    -0.150, 0, 0, //tof2
    -0.065, -0.047, 0, //tof3
    0.065, -0.047, 0, //tof4
    0.150, 0, 0, //tof5
    0.065, 0.047, 0; //tof6


// // Cordinates after shifting 2,3 and 5,6 on the robot
// utils::Matrix<double, 6, 3> tof_coordinates; 
// tof_coordinates << -0.065,  0.047, 0, //tof1
//                    -0.065, -0.047, 0, //tof2
//                    -0.150,      0, 0, //tof3
//                     0.065, -0.047, 0, //tof4
//                     0.065,  0.047, 0, //tof5
//                     0.150,      0, 0; //tof6

  assert(tof_coordinates.hasNaN() != true && "Invalid! getTofCoordinates output Matrix has NaN element");

  return tof_coordinates;
}