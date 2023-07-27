#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <array>
#include <pybind11/stl.h>

#include "kinematics/serial3r_kinematics.h"
namespace py = pybind11;
using namespace std;

// bindings to Serial3RKinematics class
PYBIND11_MODULE(Serial3RKinematics, m) {
  // optional module docstring
  m.doc() = "pybind11 trajectory plugin";
  py::class_<kine::Serial3RKinematics>(m, "Serial3RKinematics")
    .def(py::init<std::vector<double>>())
    .def(py::init<>())
    .def("turnOnSafety", &kine::Serial3RKinematics::turnOnSafety)
    .def("turnOffSafety", &kine::Serial3RKinematics::turnOffSafety)
    .def("getSafety", &kine::Serial3RKinematics::getSafety)
    .def("searchSafePosition", &kine::Serial3RKinematics::searchSafePosition)
    .def("inWorkspace", &kine::Serial3RKinematics::inWorkspace)
    .def("forwardKinematics", static_cast<void(kine::Serial3RKinematics::*)(std::string, std::vector<double>, std::vector<double>&)>(&kine::Serial3RKinematics::forwardKinematics))
    .def("forwardKinematics", static_cast<std::vector<double>(kine::Serial3RKinematics::*)(std::string, std::vector<double>)>(&kine::Serial3RKinematics::forwardKinematics))
    .def("inverseKinematics", static_cast<int(kine::Serial3RKinematics::*)(std::string, std::vector<double>, char, std::vector<double>&)>(&kine::Serial3RKinematics::inverseKinematics))
    .def("inverseKinematics", static_cast<std::vector<double>(kine::Serial3RKinematics::*)(std::string, std::vector<double>, char)>(&kine::Serial3RKinematics::inverseKinematics));
  // Functions invereKinematics and forwardKinematics have been overloaded because there is an issue of pybind not handling arguments passed by reference.
  //Compiled for C++11
}
