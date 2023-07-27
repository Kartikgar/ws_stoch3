# Install docker
Install docker from [here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)

# Add USER to docker group (to run docker without sudo everytime)
```
$ sudo groupadd docker
$ sudo gpasswd -a ${USER} docker
$ sudo service docker restart
```

# Enable GUI control access
```
$ xauth list
$ xhost +local:root (to give control)
$ xhost -local:root (to remove control)
```

# Install nviadia-docker2 
[Reference1](https://nvidia.github.io/libnvidia-container/)
[Reference2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
```
$ DIST=$(. /etc/os-release; echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | \sudo apt-key add -
$ curl -s -L https://nvidia.github.io/libnvidia-container/$DIST/libnvidia-container.list | \sudo tee /etc/apt/sources.list.d/libnvidia-container.list
$ sudo apt-get update
$ sudo apt-get install nvidia-docker2
```

# Install VSCode
Install VSCode from [here](https://code.visualstudio.com/download). After this install the dev container extension inside VSCode, you can follow 
along [this](https://code.visualstudio.com/docs/devcontainers/tutorial) tutorial to get familiar with it. As a side stuff, also play around with 
the [Python Tutorial](https://github.com/microsoft/vscode-remote-try-python) of Dev container.

# Setting up Dev Container for ROS
With the help of devcontainer, you can develop and test your code inside a docker container and it gets automatically saved on your 
local files system. This and some other benefits associated with it make it a powerful tool for developing your code without tampering any system
libraries.
The below instructions are meant for ROS Melodic version on Ubuntu 18.04, but these can be easily used for other versions of ROS/Ubuntu by 
appropriately changing the commands.

**Note:**
- **Before starting ensure you have enough disk space available wherever you have installed docker (in `root` or `home`)**
- **These instructions have been tested on a system with `Nvidia GPUs`, for any other variants  like `Integrated Intel GPUs` or `AMD GPUs`, 
these might not work**

1. Create a workspace folder where you would like to develop the code on your local system
2. Create a folder called `src`, and clone this repository inside that folder
3. Then create a folder called `~/<Workspace Name>/.devcontainer`
4. Create the following 3 files inside the `.devcontainer` folder `devcontainer.json`, `Dockerfile` (no extension), `post_create_commands.sh`

Now populate the contents of the following files as follows and carefully read each line:

*Note: I am assuming the workspace folder name is `ws_stoch3` for the content of the files below*

#### devcontainer.json
```json
// For format details, see https://aka.ms/devcontainer.json. For config options, see the README at:
// https://github.com/microsoft/vscode-dev-containers/tree/v0.224.2/containers/ubuntu
{
	"name": "Stoch3",
	"build": {
		"dockerfile": "Dockerfile",
		"context": "..", 
		// Update 'VARIANT' to pick an Ubuntu version: hirsute, focal, bionic
		// Use hirsute or bionic on local arm64/Apple Silicon.
		"args": { "VARIANT": "bionic" }
	},

	"runArgs": [
		"--name","ws_stoch3", // Replace with your workspace folder name 
		// Bind resources needed for X11 display to work properly.
		"-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
		"-e", "DISPLAY",
		// Specify nvidia GPU options: seperate argument from its value
		"--gpus", "all",
		"--ipc", "host",
		"-v", "/dev/input/js0:/dev/input/js0:rw", // For joystick
		"--network", "host",
		"--privileged"
	],

	// Set *default* container specific settings.json values on container create.
	"settings": { 
		"python.defaultInterpreterPath": "/usr/bin/python"
	},

	// You may skip the installation of the extensions below by simply commenting each entry
	// Add the IDs of extensions you want installed when the container is created.
	"extensions": [
		"ms-vscode.cpptools",
		"twxs.cmake",
		"github.copilot",
		"ms-python.vscode-pylance",
		"ms-python.python",
		"VisualStudioExptTeam.vscodeintellicode",
		"ms-vscode.cpptools-extension-pack"
	],

	// Use 'forwardPorts' to make a list of ports inside the container available locally.
	// "forwardPorts": [],

	
	// Use 'postCreateCommand' to run commands after the container is created.
	"postCreateCommand": "bash .devcontainer/post_create_commands.sh",

	// Comment out to connect as root instead. More info: https://aka.ms/vscode-remote/containers/non-root.
	"remoteUser": "vscode"
}
```

#### Dockerfile
```Dockerfile
FROM osrf/ros:melodic-desktop-full

# For hardware accelaration
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display

### add non-root user
RUN useradd -m vscode && \
    echo vscode ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/vscode && \
    chmod 0440 /etc/sudoers.d/vscode

ENV SHELL /bin/bash

### init rosdep
RUN rosdep update

RUN apt-get update && apt-get install -y apt-utils build-essential

### For all ros control packages
RUN apt-get update && apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers -y

RUN apt-get update && apt-get install -q -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-$ROS_DISTRO-hector-gazebo-plugins
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-velodyne*
RUN apt-get install -y ros-$ROS_DISTRO-pointcloud-to-laserscan ros-$ROS_DISTRO-twist-mux ros-$ROS_DISTRO-robot-localization

### For GUI rendering apps
RUN apt-get install dbus libxext6 libxrender1 libxtst6 libxi6 libxxf86vm1 mesa-utils python3-pip x11-apps xterm iputils-ping -y

### For Video Rendering
RUN apt-get install -y ffmpeg

### For qpSWIFT Python
# RUN git clone https://github.com/qpSWIFT/qpSWIFT.git
# WORKDIR /qpSWIFT/python
# RUN python3 setup.py install

### Some basic python libraries
# RUN pip3 install --upgrade pip
# RUN pip3 install PyYAML
# RUN pip3 install scipy
# RUN pip3 install matplotlib

### For Stoch3 Env Pybullet or RF-MPC branch of stoch3_ros
# RUN apt-get install -y libeigen3-dev python3-tk
# RUN pip3 install --upgrade cmake
# RUN pip3 install pybind11[global]
# RUN pip3 install gym
# RUN pip3 install pybullet
# RUN pip3 install pandas

### For Pytorch CPU version
# RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu

### If you want to use google benchmark library
# WORKDIR /
# RUN git clone https://github.com/google/benchmark.git
# WORKDIR /benchmark
# RUN cmake -E make_directory "build"
# WORKDIR /benchmark/build
# RUN cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../
# WORKDIR /benchmark
# RUN cmake --build "build" --config Release --target install
```

#### post_create_commands.sh
```sh
#! bin/bash

### Note all these commands are inside devcontainer and not your local system

# Fix Qt rendering bugs
echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc
echo "export QT_GRAPHICSSYSTEM="native"" >> ~/.bashrc

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Stoch3 specific
echo "source src/stoch3_ros/scripts/setup.sh" >> ~/.bashrc

source ~/.bashrc

# rosdep update
# rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
# cd /workspaces/ws_stoch3/ 
# rosdep install -r --ignore-src --from-paths src -y

# qpSWIFT C++ install
cd /workspaces/ws_stoch3/src/stoch3_ros/qpSWIFT/build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install
```

# Launching Dev Container

Open your local workspace folder in VSCode, then automatically you should be able to see a pop-up window asking to "re-open" this folder in a
devcontainer. If the pop-up window does not appear, you can manually open it in a devcontainer by clicking on the button at bottom left corner.

# Troubleshooting

- If any error with XAUTH, create a empty file named ".docker.xauth" in /tmp directory

- If runtime error related to nvidia, then run the following commands
```
$ sudo systemctl daemon-reload
$ sudo systemctl restart docker
```

If face any problems, please feel free to open up an issue and ping me @aditya-shirwatkar
