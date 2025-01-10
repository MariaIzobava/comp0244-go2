# Tutorial of Environment Setup of Unitree GO2

## Overview

This repository provides an environment that can be run using Docker. The environment is designed to run specific software or tasks, and the following instructions will guide you through installing dependencies, setting up the Docker container, and running the necessary files.

## Installation on Ubuntu 20+
### Step 1: Open a terminal and clone the repo:
```bash
mkdir /home/$USER/comp0244_ws
cd /home/$USER/comp0244_ws
git clone --recursive git@github.com:COMP0244-S25/comp0244-go2-sol.git
```

#### Environment: ROS2-Humble
##### Step 2: In the same terminal, download the Docker (https://docs.docker.com/engine/install):
```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

##### Step 3: In the same terminal, download the docker images (click [here](https://aws.amazon.com/docker) to understand docker):
```bash
sudo docker pull jjiao/comp0244:unitree-go-ros2-humble
sudo docker tag jjiao/comp0244:unitree-go-ros2-humble comp0244:unitree-go-ros2-humble
```

##### Step 4: In the same terminal, enable the display:
```bash
sudo apt-get install x11-xserver-utils
xhost +
```
##### Step 5: In the same terminal, create the docker container:
```bash
sudo docker run -it -e DISPLAY -e QT_X11_NO_MITSHM=1 -e XAUTHORITY=/tmp/.docker.xauth \
-v /home/$USER/comp0244_ws:/usr/app/comp0244_ws \
--network host \
--name comp0244_unitree comp0244:unitree-go-ros2-humble /bin/bash
```
