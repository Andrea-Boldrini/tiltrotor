# Simulation of a Tilt-Rotor UAV
This package consists of a tilt-rotor UAV simulation in the ROS Gazebo environment, integrated with the PX4 controller. It is developed in Ubuntu 20.04, ROS Noetic and Gazebo 11.

Moreover, the folders `position_attitude_loops_ODEs` and `position_attitude_loops_Simscape` contain Matlab/Simulink files which allow to run simulations in the relative environment in a simpler, but less advanced fashion.
# Guide for Ros/Gazebo simulation
## Setup of the environment
1. Follow the [ROS 1 with MAVROS Installation Guide](https://docs.px4.io/main/en/ros/mavros_installation.html) to install three main software, PX4, ROS 1 and MAVROS, with the recommended modifications below:
   - for PX4: while running `ubuntu.sh`, remove `--no-sim-tools --no-nuttx` options from the suggested command line in the original website;
   - for MAVROS: when installing MAVROS, follow the instructions of the source installation steps, instead of the binary ones. Moreover, it is required to install catkin_tools:
     ```plaintext
      sudo apt-get install python3-catkin-tools
      ```
      Then, note that `noetic mavlink` should be downloaded instead of `kinetic mavlink`, replacing `kinetic` with `noetic` in the relative installation line, and the `released/stable` version of MAVROS has to be preferred to the `latest source`.

   After installing everything, update the `.bashrc` file:
   ```plaintext
   source ~/.../catkin_ws/devel/setup.bash
   source ~/.../PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/.../PX4-Autopilot ~/.../PX4-Autopilot/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/.../PX4-Autopilot
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/.../PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
   ```
   replacing `...` with your directory path.

2. Download this repository in your computer:
   ```plaintext
   git clone https://github.com/Andrea-Boldrini/tiltrotor.git
   ```
3. Copy the contents of `models` folder into Gazebo's model directory `/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`. 

4. Copy the `models_plugin` folder and locate it in the src folder located in the catkin_ws folder.

5. Copy the `10020_gazebo-classic_iris_tiltrotor` file and locate into directory `/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`. Moreover, modify the cmake file `CMakeLists.txt` in the same directory adding the line:
   ```plaintext
   10020_gazebo-classic_iris_tiltrotor
   ```
6. Modify the `<physics>` section of the `empty.world` in directory `/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds` file as below:
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
7. Open a new terminal and build the package with the command:
   ```plaintext
   cd PX4-Autopilot
   DONT_RUN=1 make px4_sitl gazebo-classic
   ```
8. Choose between `tiltrotor_drone_PID`, `tiltrotor_drone_LQR`, `tiltrotor_drone_LMPC` and `tiltrotor_drone_NLMPC` folders, which are the controllers developed for the tilt-rotor UAV respectively using PID, PID+LQR, linear MPC and nonlinear MPC. Then rename the selected folder as `tiltrotor_drone` and locate it in the src folder located in the catkin_ws folder.
   
9. In order to apply external disturbances on the UAV, it is necessary to install the external wrench emulator `https://github.com/joshuataylor00/gazebo_wrench_emulator.git`, following the instructions in the related repository.
   
10. When using `tiltrotor_drone_LMPC` and `tiltrotor_drone_NLMPC` it is necessary to download the open-source tools for nonlinear optimization `qpOASES` 
    ```
    git clone https://github.com/coin-or/qpOASES.git
    cd qpOASES
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    ```
    and `CasADi`:
    ```
    git clone https://github.com/casadi/casadi.git
    cd casadi
    mkdir build && cd build
    sudo apt install swig
    cmake .. -
      -DWITH_QPOASES=ON -
      -DWITH_LAPACK=ON -
      -DWITH_PYTHON=ON -
      -DWITH_STATIC_CASADI=OFF -
      -DWITH_EXAMPLES=ON -
      -DPYTHON_EXECUTABLE=$(which python3) -
      -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    ```
    Finally, update the `.bashrc` file exporting the relative libraries:
    ```
    export LD_LIBRARY_PATH=~/.../casadi/build/lib:~/.../qpoases/build/libs:$LD_LIBRARY_PATH
    ```
## Start of the simulation
Build the environment:
```
cd catkin_ws
catkin build
```
Open three terminals. Use the first one to launch PX4, ROS and Gazebo:
```
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_tiltrotor
```
the second one to launch the flight code to control the UAV:
```
roslaunch tiltrotor_drone flight_controller.launch
```
and the last one to dynamically modify the setpoints of the UAV:
```
rosrun rqt_reconfigure rqt_reconfigure
```
When changing setpoint, note that:
- position commands are expressed in vehicle frame, therefore x and y are in the plane parallel to the ground passing through the CoG of the UAV, with the x direction aligned with the yaw, while z is perpendicular to that plane in upward direction;
- change of yaw modifies the desired x and y position in vehicle frame, so it necessary that yaw reaches desired setpoint before making any other maneuver.

# Guide for Matlab/Simulink simulation
`position_attitude_loops_ODEs` simulates the UAV directly integrating the ODEs that describe the system, while `position_attitude_loops_Simscape` exploits a Simscape model of the tilt-rotor UAV to run the simulations.

It is possible to choose between different Simulink modellings:
- `model_3` (PID), where all directions are controlled with PID;
- `model_4` (LQR+PID), where x, z and pitch are controlled with LQR, and y and yaw with PID;
- `model_5` (LQR+PID), where x, z and pitch are controlled with LQR, and y and yaw with PID;
- `model_6` (MPC+PID), where x, z and pitch are controlled with MPC, and y and yaw with PID;
- `model_7` (MPC), where all directions are controlled with MPC.

The first four models are available in both the folders, while the last one just in the latter.

To run the simulations it is firstly necessary to initialize the simulation and controller parameters running the relative codes. 
For the first folder (ODEs):
- `controller_parameters_model_3_and_4` initializes the PID parameters for `model_3` and `model_4`; 
- `controller_parameters_model_5_and_6` initializes the PID parameters for `model_5` and `model_6`; 
- `LQR_controller_pitching` initializes the LQR controller for `model_4` and `model_5`; 
- `MPC_controller_pitching` initializes the MPC controller for `model_6`;
- `simulation_parameters` initializes simulation parameters and reference setpoint (that can be updated as desired).

For the other folder (Simscape):
- `controller_parameters` initializes the PID parameters (follow the instructions at the head of the file to choose the right values);
- `LQR_controller_pitching` initializes the LQR controller for `model_4` and `model_5`; 
- `MPC_controller_pitching` initializes the MPC controller for `model_6`;
- `MPC_controller_full` initializes the MPC controller for `model_7`;
- `simulation_parameters` initializes simulation parameters and reference setpoint (that can be updated as desired).

In the Simscape simulations of `model_6` and `model_7` it is possible to apply an external wrench: for the former it is necessary to set the desired values in the `simulation_parameters` file, while for the latter the values have to be set directly in Simulink with the command toolbar.

After every simulation it is possible to plot trajectory, input values and other useful variables using the relative file for visualization. In particular, for the first folder (ODEs):
- `plots` should be run to plot trajectory with respect to time, velocity, tilt angle and rotors thrust for all models.

For the other folder (Simscape):
- `integration_plot` should be run to plot trajectory with respect to time, velocity, tilt angle and rotors thrust for all models, external wrench estimation for `model_6`, external wrench estimation for `model_7` and velocity for `model_5`.
