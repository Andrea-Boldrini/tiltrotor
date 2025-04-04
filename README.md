# Simulation of a Tilt-Rotor UAV
This package consists a tilt-rotor UAV simulation in the ROS Gazebo environment, integrated with the PX4 controller. It is developed in Ubuntu 20.04, ROS Noetic and Gazebo 11.
# Guide
## Setup of the environment
Install ROS 1 and MAVROS following the ROS 1 with MAVROS Installation Guide (https://docs.px4.io/main/en/ros/mavros_installation.html).

Download the PX4-Autopilot uploaded in this repository, which is already modified to handle tilt-rotor UAV simulations.

Download the tiltrotor_drone_PID or tiltrotor_drone_LQR folders, which are the controllers developed for the tilt-rotor UAV respectively using PID and PID+LQR. Then rename the selected folder as tiltrotor_drone and locate it in the src folder located in the catkin_ws folder.
## Start of the simulation
Build the environment (cd catkin_ws, catkin build).
Open three terminals. Use the first one to launch the flight code to control the UAV (cd catkin_ws, roslaunch tiltrotor_drone flight_controller.launch), the second one to launch PX4, ROS and Gazebo (cd PX4-Autopilot, roslaunch launch/mavros_posix_sitl.launch) and the last one to dynamically modify the setpoints of the UAV (rosrun rqt_reconfigure rqt_reconfigure).
