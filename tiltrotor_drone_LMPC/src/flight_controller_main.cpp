/**
 * package:  tiltrotor_drone
 * node:     flight_controller
 * file:     flight_controller_main.cpp
 * authors:  Efe Camci
             Yun Ting Chen
             Andrea Boldrini
 * company:  Institute for Infocomm Research (I2R), A*STAR Singapore
 * date:     Jan 2025
 *
 * @copyright
 * Copyright (C) 2025.
 */

#include <flight_controller.h>
#include <mpc_solver.hpp>
#include <wrench_estimator.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flight_controller");
    ros::NodeHandle nh;
    
    // Initialize the object
    FlightController FC;
    FC.initRosComms(&nh);
    FC.initParams();
   
    // Set the node rate
    int node_rate;
    nh.param<int>("/flight_controller/node_rate", node_rate, 50);
    ros::Rate rate(node_rate);

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
