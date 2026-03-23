Setup Tips and Instructions for the Tello Drone:

Lab Tello Drone is: 5FD2E9

WSL issues:

1. May need to run this command to make sure Gazebo runs with the proper renderer for WSL.
´´´
export LIBGL_ALWAYS_SOFTWARE=1
´´´
2. Sometimes Gazebo closes with a process running that need to be killed.
´´´
pkill -9 -f gazebo
´´´

Other Troubleshooting:

1. Set Domain ID to 6, and source the project:
´´´
export ROS_DOMAIN_ID=6
source install/setup.bash
´´´

2. Code to run to build the project (Tello)
´´´ 
colcon build --symlink-install
´´´

Additional Tips from the slides:
1. Robot Camera needs to consider a 250pixel offset due to it's placement
2. Running the robot requires using 2 terminals one runnning
´´´
ros2 launch tello_driver teleop_launch.py
´´´
and a second can then send the commands/algorithm to the drone
´´´
ros2 service call /tello_action tello_msgs/TelloActon "{cmd: 'takeoff'}"
´´´
