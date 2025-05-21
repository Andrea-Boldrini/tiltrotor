# Simulation of a Tilt-Rotor UAV
This package consists of a tilt-rotor UAV simulation in the ROS Gazebo environment, integrated with the PX4 controller. It is developed in Ubuntu 20.04, ROS Noetic and Gazebo 11.

Moreover, the folders `position_attitude_loops_ODEs` and `position_attitude_loops_Simscape` contain Matlab/Simulink files which allow to run simulations in the relative environment in a simpler, but less advance fashion.
# Guide for Ros/Gazebo simulation
## Setup of the environment
Install ROS 1, PX4 and MAVROS following the [ROS 1 with MAVROS Installation Guide](https://docs.px4.io/main/en/ros/mavros_installation.html).

Download the `models` folder and copy its components into Gazebo's model directory `/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`. Download the `models_plugin` folder and locate it in the src folder located in the catkin_ws folder.

Modify launch files `px4.launch` `mavros_posix_sitl.launch` in directory ```launch```, changing vehicle name from `iris` to `iris_tiltrotor`.

Ensure the `empty.world` in directory `/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds` file includes the following physics parameters:

```plaintext
<physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <max_contacts>250</max_contacts>
      <ode>
        <solver>
          <type>quick</type>
          <iters>40</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>
```
Install QGroundControl and, when running simulation, launch it and set the following parameters: 
```plaintext
EKF2_EV_CTRL = 15
EKF2_HGT_REF = VISION
```
Download the `tiltrotor_drone_PID` or `tiltrotor_drone_LQR` or `tiltrotor_drone_LMPC` or `tiltrotor_drone_NLMPC` folders, which are the controllers developed for the tilt-rotor UAV respectively using PID, PID+LQR, linear MPC and nonlinear MPC. Then rename the selected folder as `tiltrotor_drone` and locate it in the src folder located in the catkin_ws folder.

When using `tiltrotor_drone_LMPC` and `tiltrotor_drone_NLMPC` it is necessary to download the external wrench emulator `https://github.com/joshuataylor00/gazebo_wrench_emulator.git`, following the instructions in the relative repository, in order to apply external disturbances on the UAV. 
Moreover, it is required to download the open-source tools for nonlinear optimization `CasADi`
```
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build && cd build
cmake .. -DENABLE_QPOASES=ON -DQP_OASES_ROOT_DIR=/usr/local
make -j$(nproc)
sudo make install
```
and `qpOASES`:
```
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
Finally, update the `.bashrc` file exporting the relative libraries:
```
export LD_LIBRARY_PATH=/.../casadi/build/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/.../qpoases/build/libs:$LD_LIBRARY_PATH
```
## Start of the simulation
Build the environment:
```
cd catkin_ws
catkin build
```
Open three terminals. Use the first one to launch the flight code to control the UAV:
```
cd catkin_ws
roslaunch tiltrotor_drone flight_controller.launch
```
the second one to launch PX4, ROS and Gazebo:
```
cd PX4-Autopilot
roslaunch launch/mavros_posix_sitl.launch
```
and the last one to dynamically modify the setpoints of the UAV:
```
rosrun rqt_reconfigure rqt_reconfigure
```
# Guide for Matlab/Simulink simulation
`position_attitude_loops_ODEs` simulates the UAV directly integrating the ODEs that describe the system, while `position_attitude_loops_Simscape` exploits a Simscape model of the tilt-rotor UAV to run the simulations.

In both the folders it is possible to choose between four different Simulink modellings, named `model_3`, `model_4`, `model_5` and `model_6`, related to four different control schemes (respectively PID, LQR+PID, LQR+PID and linear MPC+PID). In `position_attitude_loops_Simscape` it is also avaible another linear MPC implementation, called `model_7`.

To run the simulations it is firstly necessary to initialize the simulation and controller parameters, running the relative codes. Before the simulation with `model_4` and `model_5`, which rely also on LQR, it necessary to run `LQR_controller_pitching` to define the optimal control gain, while before the simulation with `model_6` and `model_7`, which rely on MPC, it is required to run respectively `MPC_controller_pitching` and  `MPC_controller_full`. 

After every simulation it is possible to plot trajectory, input values and other useful variables using the relative file for visualization. 
