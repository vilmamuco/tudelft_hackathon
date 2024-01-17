
## Acknowledgements

This repository is a simplified version of the REMARO Summer School Delft 2022 - Underwater robotics hackathon. For more info, please visit: <a href="https://github.com/remaro-network/tudelft_hackathon.git"> remaro-network/tudelft_hackathon

## Setup

Tested with:
- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo (Ignition) Garden](https://gazebosim.org/docs/garden/install_ubuntu)
- [ArduPilot (Sub-4.1)](https://github.com/ArduPilot/ardupilot/tree/f2af3c7ed2907be914c41d8512654a77498d3870)
- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)
- [mavros2](https://github.com/mavlink/mavros)
- [remaro_world](https://github.com/remaro-network/remaro_worlds/tree/ign-garden)
- [bluerov2_ignition](https://github.com/Rezenders/bluerov2_ignition)


## Installation

**Disclaimer**
Running docker images with graphical user interface is a little bit trick and might not work in all systems.
We tested in a system with ubuntu 22.04 and with a NVIDIA gpu.
It might not work on systems with AMD gpus, and on MAC.

### Install prerequisites to run with docker

- Install docker on your machine. You can find instructions [here](https://docs.docker.com/engine/install/ubuntu/)
- Allow non-root users to manage docker. Instructions [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
- Install VSCode. Instructions [here](https://code.visualstudio.com/download)
- Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)(only needed if you have a nvidia GPU)


## Run it with docker via CLI

Create docker network:

```Bash
sudo docker network create ros_net
```

### Run Ignition simulation + ardupilot SITL:

If you a NVIDIA GPU:
```Bash
xhost +local:root
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --gpus all ghcr.io/remaro-network/tudelft_hackathon:nvidia ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

If you have an AMD GPU:
```Bash
xhost +local:root ;
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri --group-add video  ghcr.io/remaro-network/tudelft_hackathon:non-nvidia ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

If you have an Intel GPU:
```Bash
xhost +local:root ;
sudo docker run -it --rm --name ignition --net ros_net -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri  ghcr.io/remaro-network/tudelft_hackathon:non-nvidia ros2 launch tudelft_hackathon bluerov_ign_sim.launch.py ardusub:=true mavros_url:='bluerov:14551'
```

### Development with docker via cli

To add your modifications into the docker images you need to rebuild the relevant docker images.
In this case, run the `build-dev-images.sh` script to rebuild them. And make sure to substitute in the `docker run` commands the images from rezenders to you local images. I.e: `rezenders/ignition:hackathon` -> `ignition:hackathon-dev` and  `rezenders/ros-foxy-hackathon` -> `ros-foxy-hackathon:dev`

## Run it with docker with VSCode

Check instructions [here](https://github.com/remaro-network/tudelft_hackathon/blob/ros2/dockerfiles/ros1-2-ignition/README.md)

## Run it locally

### Simulation
Before running anything you need to source the workspace. With this command:

```Bash
source ~/tudelft_hackathon_ws/install/setup.bash
```

Or you can add that to the ~/.bashrc file to prevent needing to source everytime.

```Bash
echo "source ~/tudelft_hackathon_ws/install/setup.bash" >> ~/.bashrc
```
Don't forget to re-open your terminal after altering the `~/.bashrc` file.

In one terminal run ardusub SITL:
```Bash
  sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

In another terminal run the simulation + mavros + agent:
```Bash
 ros2 launch tudelft_hackathon bluerov_bringup.launch.py simulation:=true ardusub:=false
```


## Explanation

Simplified system architecture:

![System architecture](https://user-images.githubusercontent.com/20564040/174649275-41a2d0bd-54ed-485f-bfcb-36ffe94dd11c.png)

The system was designed to be used both with a real or simulated BlueROV2. When
used with a simulation, the left nodes are deployed. And when used with the real
robot the right nodes are deployed.The agent and MAVROS nodes are always deployed.

First, let's take a look on how the real BlueROV2 works, then we see how the simulation
is setup to mimic it.


### Simulated BlueROV2

**Simulated BlueROV2:** To simulate the BlueROV2 we are using Gazebo (Ignition). Unfortunately, until the moment of writing this readme, there is no sonar plugin for Ignition. Thus, we are using a lidar plugin instead, configured to have the same speed and measurement range as the Ping360 sonar. The bluerov2 model is [here](https://github.com/Rezenders/bluerov2_ignition/blob/main/models/bluerov2/model.sdf) and the bluerov2 model with a lidar is [here](https://github.com/Rezenders/bluerov2_ignition/blob/main/models/bluerov2_lidar/model.sdf). The world being used for the simulation can be found [here](https://github.com/remaro-network/remaro_worlds/blob/ign-garden/worlds/room_walls.world), note that there is a buoyancy plugin that sets the "water" density to 1000kg/m3.

**Simulated "sonar" bridge:**  In order to have access to the simulated lidar data with ROS2 we run a bridge between ignition transport and ROS2. Something like this:

Via command line:
```
ros2 run ros_gz_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan -r /lidar:=/scan
```

With launch file:
```
package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
    remappings=[('/lidar','/scan')],
    output='screen'
```

Note that the topic where the lidar data is published has the same name (`/scan`) and type (`sensor_msgs/LaserScan`) as the topic published by the ping360 driver.

**ArduSub Sofware In The Loop (SITL):** Since when running the simulation we don't have a board with an autopilot installed, we simulate Ardusub as a SITL.

**Ardupilot gazebo plugin:** Bridge between Ignition and Ardusub. More info can be found [here](https://gazebosim.org/api/gazebo/7.0/ardupilot.html).

**MAVROS:** The only difference is that for the simulation we need to use a different `fcu_url`. In this case, `udp://:14551@:14555`.

**Agent:** Since all the interfaces are the same, the agent nodes are the same for both simulation and the real robot.
