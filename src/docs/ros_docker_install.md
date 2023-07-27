# Install docker
Install docker from [here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)

# Add USER to docker group (to run docker without sudo everytime)
```
$ sudo groupadd docker
$ sudo gpasswd -a ${USER} docker
$ sudo service docker restart
```

# Useful docker commands
```
$ docker images (Shows all the images in docker)
$ docker pull <package_name:tag> (To install a package in docker (default tag: latest))
$ docker ps -a (shows all the containers that have been created)
$ docker rm process_name (to kill a container)
$ docker image rm package_name:tag (to remove a package)
$ docker commit -m "What you did to the image" -a "Author Name" container_id repository/new_image_name (to commit changes)
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

# Install ROS package
[Reference](https://varhowto.com/install-ros-noetic-docker/)
```
$ docker pull osrf/ros:melodic-desktop-full (ros + gazebo)
$ docker run -it osrf/ros:melodic-desktop-full (to run docker package in interactive mode)
$ source /opt/ros/melodic/setup.bash
$ sudo apt-get update
```

# Install hector-gazebo plugins
```
$ sudo apt-get install -y ros-melodic-hector-gazebo-plugins
```

# Install xterm (for controlling the robot using keyboard)
```
$ sudo apt-get install -y xterm
```

# Run ros docker container with dispaly enabled
```
$ docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:melodic-desktop-full
$ mkdir home/ws_stoch3
$ cd home/ws_stoch3
$ git clone --recursive https://github.com/StochLab/stoch3_ros src (Enter your username, api token as your password)
$ source src/scripts/setup.sh
$ catkin_make or cm
$ docker commit container_id stoch3_ros (run this outside the docker container)
```
# To open another terminal in the same docker session
[Reference](https://www.cloudbees.com/blog/introductory-how-to-examples-docker-exec)
```
$ docker exec -it container_id bash
(-i (interactive) keeps stdin open and -t (tty) allocates a pseudo-terminal)
```
# If error in opening xterm:
```
$If the following error:
	No protocol specified
	Warning: This program is an suid-root program or is being run by the root user.
	The full text of the error or warning message cannot be safely formatted
	in this environment. You may get a more descriptive message by running the
	program as a non-root user or by removing the suid bit on the executable.
	xterm: Xt error: Can't open display: %s
Run the following command outside docker container
$ xhost +local:
```

# Using Hardware Acceleration with Docker
[Reference](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration)
```
$ mkdir my_melodic_image
$ cd my_melodic_image
$ touch Dockerfile
```

paste the following content

```
FROM stoch3_ros
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
		${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
		${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
```

```
$ docker build -t my_melodic_image . (Now the docker images should show a new image my_melodic_image)
```

create a script to run the image called run_my_image.bash

```
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
		xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
		if [ ! -z "$xauth_list" ]
		then
				echo $xauth_list | xauth -f $XAUTH nmerge -
		else
				touch $XAUTH
		fi
		chmod a+r $XAUTH
fi

docker run -it \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="XAUTHORITY=$XAUTH" \
	--volume="$XAUTH:$XAUTH" \
	--runtime=nvidia \
	my_melodic_image \
	bash
```

```
$ ./run_my_image.bash (Run the image)
```

If any error with XAUTH, create a empty file named ".docker.xauth" in /tmp directory
If runtime error related to nvidia, then run the following commands
```
$ sudo systemctl daemon-reload
$ sudo systemctl restart docker
```
