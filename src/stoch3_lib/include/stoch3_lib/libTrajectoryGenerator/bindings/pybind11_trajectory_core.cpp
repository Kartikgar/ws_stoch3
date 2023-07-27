#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>

#include "trajectory_generators/trajectory_core.h"

namespace py = pybind11;
using namespace std;
using namespace trajectory;
// bindings to TrajectoryCore class
PYBIND11_MODULE(TrajectoryCore, m) {

  m.doc() = "pybind11 trajectory plugin";    // optional module docstring
  py::class_<trajectory::TrajectoryCore >(m, "TrajectoryCore")
    .def(py::init<>())   //Constructor
    .def("initializeRobot", &TrajectoryCore::initializeRobot)             // Functions
    .def("setRobotParam", &TrajectoryCore::setRobotParam)
    .def("setRobotDimensions", &TrajectoryCore::setRobotDimensions)
    .def("setRobotLinkLengths", &TrajectoryCore::setRobotLinkLengths)
    .def("getRobotDimensions", &TrajectoryCore::getRobotDimensions)
    .def("getRobotLinkLengths", &TrajectoryCore::getRobotLinkLengths)
    .def("setFreq", &TrajectoryCore::setFreq)
    .def("getFreq", &TrajectoryCore::getFreq)
    .def("printConfigs", &TrajectoryCore::printConfigs)
    .def("setGaitConfig", &TrajectoryCore::setGaitConfig)
    .def("setGaitConfig2", &TrajectoryCore::setGaitConfig2)
    .def("setGaitType", &TrajectoryCore::setGaitType)
    .def("setPhase", &TrajectoryCore::setPhase)
    .def("setWalkingHeight", &TrajectoryCore::setWalkingHeight)
    .def("setSwingHeight", &TrajectoryCore::setSwingHeight)
    .def("setMaxLinearXVel", &TrajectoryCore::setMaxLinearXVel)
    .def("setMaxLinearYVel", &TrajectoryCore::setMaxLinearYVel)
    .def("setMaxAngularZVel", &TrajectoryCore::setMaxAngularZVel)
    .def("getGaitType", &TrajectoryCore::getGaitType)
    .def("getPhase", &TrajectoryCore::getPhase)
    .def("getWalkingHeight", &TrajectoryCore::getWalkingHeight)
    .def("getSwingHeight", &TrajectoryCore::getSwingHeight)
    .def("getMaxLinearXVel", &TrajectoryCore::getMaxLinearXVel)
    .def("getMaxLinearYVel", &TrajectoryCore::getMaxLinearYVel)
    .def("getMaxAngularZVel", &TrajectoryCore::getMaxAngularZVel)
    .def("constrainTheta", &TrajectoryCore::constrainTheta)
    .def("resetTheta", &TrajectoryCore::resetTheta)
    .def("getTheta", &TrajectoryCore::getTheta)
    .def("generateTrajectoryLeg", &TrajectoryCore::generateTrajectoryLeg)
    .def("generateTrajectory", &TrajectoryCore::generateTrajectory)
    .def("calculatePlanarTraj", &TrajectoryCore::calculatePlanarTraj);

}
