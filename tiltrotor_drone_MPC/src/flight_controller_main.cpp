/**
 * package:  tiltrotor_drone
 * node:     flight_controller
 * file:     flight_controller_main.cpp
 * authors:  Efe Camci
             Yun Ting Chen
 * company:  Institute for Infocomm Research (I2R), A*STAR Singapore
 * date:     Jan 2025
 *
 * @copyright
 * Copyright (C) 2025.
 */

#include <flight_controller.h>
#include <mpc_solver.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flight_controller");
    ros::NodeHandle nh;
    
    // Initialize the object
    FlightController FC;
    FC.initRosComms(&nh);
    FC.initParams();
   
    // Set the node rate
    ros::Rate rate(50);

    // Main loop
    while(ros::ok())
    {
        FC.sendCurrentDronePose();
        FC.sendCmds();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
