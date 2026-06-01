Setup Tips and Instructions for the Tello Drone:

WSL/Docker Container issues:

1. May need to run this command to make sure Gazebo runs with the proper renderer for WSL.
```
export LIBGL_ALWAYS_SOFTWARE=1
```
2. Sometimes Gazebo closes with a process running that need to be killed.
```
pkill -9 -f gazebo
```

3. Depending on the setup before going into the container the system may require giving graphics permissions to the container:
```
xhost +local:docker
```

Other Troubleshooting:
1. When initializing the system I first start the container in this case:
```
docker start ros_tello
```
then I open 3 more terminals in terminator and group them so they all copy the same input so that all 4 windows are initialized without having to re-type the code, then enter the docker container on each with:
```
docker exec -it ros_tello bash
```

2. Next we set the Domain ID to 6, and source the project (may also need to source ROS if using a container):
```
export ROS_DOMAIN_ID=6
source install/setup.bash
source /opt/ros/humble/setup.bash
```

3. Code to run to build the project (Tello), build from the main folder containing best.pt
``` 
colcon build --symlink-install
```

Instructions for drunning the drone with the current best model:
1. Running the drone requires running both the tello_driver, and the python script found within the tello_autonomy folder which loads the yolo model.
2. To get both the driver and python file to run it needs access to graphics and each script will open a window showing the drones camera view, with the Python model displaying the gates and status of the drone.

The first terminal will run the teleop_launch.py which enables the drone to communicate with and recieve inputs from the python script, this must be run first to initialize the drone:
```
ros2 launch tello_driver teleop_launch.py
```
The second terminal then needs to navigate to the tello_autonomy folder to run the python script.
```
cd /tello_autonomy/tello_autonomy
python3 vision_stop.py
```
I recommend also having a 3rd and potentially fourth terminal open, one to issue specific ROS_TELLO command as Tello_Actions and another to view the resposonse. In this scenario the 3rd terminal will run:
```
ros topic echo tello_reponse
```
This allows us to land the drone if the python script crashes or fails, check battery, or check the drones operating temperature. It is also useful if there are flight stability issues to check with takeoff first.

Then is the job of the 4th terminal which issues the commands for basic drone testing and feedback, the 4 main commands we used were:
```
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'land'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'battery?'}"
ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'temp?'}"
```
