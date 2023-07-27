# libTrajectoryGenerator

This repository is a header only library in c++ for some common functionalities required to generate gait trajectories for quadrupedal locomotion. To use them in your projects, simply add this repository as a submodule. Python bindings for TrajectoryCore and Serial3RKinematics modules is also available.

*Note:* 
- *Although this library is meant to be used for quadrupedal locomotion, the same structure can be extended to other legged systems as well*
- *The utils module can be used for any general purpose application*

## Note

The codebase has been migrated to Eigen3 for performance improvements. The input and output interfaces have been completely changed. Please update your projects to use this repository as a header only library

- `Testing time for trajectory_core in the old cpp codebase: ~1200 microseconds`
- `Testing time for trajectory_core in the new cpp codebase: ~600 microseconds `
- `Testing time for trajectory_core in the python codebase: ~69000 microseconds `

The python bindings have not been migrated to Eigen3 yet.

## Download

This package has a dependency on pybind11, which has been added as a submodule. To add all the required files, use the following command:
```
git clone --recursive https://github.com/StochLab/libTrajectoryGenerator.git
cd libTrajectoryGenerator
git submodule update --init  
```
This will have downloaded all files necessary for operation.

---
## Build

To compile the code, go into the bindings folder and make a build folder. Go into the build folder and run cmake .. and make
```
cd bindings 
mkdir build && cd build
cmake .. && make
```
The generted modules will have a .so extension, stored in the build folder

**Note:** While compiling, note the version of Python detected by cmake. On any system running conda or any python platform apart from native ubuntu, version mismatch may cause the module to not be detected. For example:

```
-- Found PythonInterp: /usr/bin/python3.6 (found version "3.6.9")
-- Found PythonLibs: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
-- Performing Test HAS_FLTO
-- Performing Test HAS_FLTO - Success
-- Found PythonLibs: /usr/lib/x86_64-linux-gnu/libpython3.6m.so (found version "3.6.9")
```
Here the version found is Python 3.6.9

---
## Python bindings usage

To import the modules, add the following import statements:
```
from build.TrajectoryCore import *
from build.Serial3RKinematics import *
```
