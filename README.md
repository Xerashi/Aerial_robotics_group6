# `Click here to go to the SETUP if you do not want to know more about the default Tello Drone environment`
- [Setup Guide](./SETUP.md)

# `tello_ros2_humble_driver`


The main tello ros driver is located in the tello_ros package. You can find the README file in the tello_ros package. Notice that the readme file in the tello_ros package is not up to date, it indicates that it's using ROS2 Eloquent, but we are using ROS2 Humble. But still, you should use the readme file to understand the core concepts of this tello ros driver, like the topics, services, etc. 

For the installation of this driver on ros2 humble, you should follow the instuctions below.

## Installation

### Install ROS 2 Humble
Follow the instructions at:
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)  
Use the `ros-humble-desktop` option for full desktop tools.

---

### Install dependencies
first install gazebo for ros humble
    
    source /opt/ros/humble/setup.bash 
    sudo apt-get install ros-${ROS_DISTRO}-ros-gz

after that
    
    sudo apt update
    sudo apt install \
    libasio-dev \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration-parsers \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros* \
    libignition-rendering6 


### Build this package

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/TIERS/tello_ros2_humble_driver.git
    cd tello_ros2_humble_driver/
    pip install -r requirements.txt
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build


### Just run the driver to control the real drone

    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch tello_driver teleop_launch.py

This is sufficient to just control the real drone. No matter you want to teleoprate with gamepad or keyboard, or you want to use the python codes to control the drone.


### Run a simulation

    cd ~/ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo simple_launch.py
    
You will see a single drone in a blank world. You can control the drone using the joystick.

### Control the drone

Notice！For real tello and simulation, the name of the topic and service might be different, you better manually check by ros2 topic or service list.


example:

    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
    ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'land'}"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/drone1

-----
If you run into the **No namespace found** error re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh


### Tello simulation in [Gazebo](http://gazebosim.org/)


you can go to /tello_ros/tello_gazebo to check more details.
`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing, and basic flight dynamics
* `markers` contains Gazebo models for fiducial markers
* `fiducial.world` is a simple world with multiple fiducial markers
* `inject_entity.py` is a script to spawn a model (URDF or SDF) in a running Gazebo instance
* the built-in camera plugin is used to emulate the Gazebo forward-facing camera

-----

## Instructions for using Docker
There is also an image prepared for this repo to streamline the process. This image also includes the [Tentone Tello-ROS2 repository](https://github.com/tentone/tello-ros2)
 for working with a real Tello.

### The Image  
To build the image, go to the root of the repo run the Docker file using the following command:

    docker build -t drone_racing_ros2:humble .

Before running Docker, make sure to tell your X11 server to allow local connections from root (the user inside the container).

    xhost +local:root

afterwards

    docker run -it --rm \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    drone_racing_ros2:humble

## Running the docker in WSL

By default, you are running WSL on Windows, be aware that WSL2 does not have direct GUI access, and Docker containers run in a separate network + user space, which blocks them from rendering GUI windows. Fixing this involves:

1. Making sure a GUI server is running on Windows
2. Granting access to WSL and Docker to use that display
3. Avoiding access control issues (xhost, DISPLAY)

### Firewall check
Before starting, make sure Tello can communicate with you through windows -> WSL -> container:
1. Go to Windows Defender Firewall > Advanced Settings > Inbound Rules
2. Create a rule for:
        UDP
        Port 11111
        Allow connection
3. Ensure your active network profile (likely Private) has that rule applied.
4. do the same for ports 8889 and 8890.

### Launch VcXsrv

1. Open `XLaunch`
2. Select:
   - ✅ Multiple windows
   - ✅ Display number: `0`
   - ✅ Start no client
   - ❌ Disable native OpenGL
   - ✅ **Disable access control**
3. Allow through Windows Firewall (Private + Public)

---

### WSL2 Configuration
First, you have to set wsl networking configuration to mirrored. To do that:

1. Go to C:\Users\{YOUR_USER_NAME}
2. Create or edit .wslconfig and add

        [wsl2]
        networkingMode=mirrored
3. Save and exit.
    
afterwards, go inside the WSL and 
```bash
export DISPLAY=127.0.0.1:0
xhost +
```
---

### Test X11

```bash
sudo apt install -y x11-apps
xeyes
```

If `xeyes` GUI opens, you're ready.

---

### Run Docker Container

```bash
docker run -it --rm \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  danms98/drone_racing_ros2:humble
```

---

## 🎮 Launch the Simulation

Inside the container:

```bash
ros2 launch tello_gazebo simple_launch.py
```




    










