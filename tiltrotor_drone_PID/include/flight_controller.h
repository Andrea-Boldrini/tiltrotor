#ifndef _FLIGHT_CONTROLLER_H
#define _FLIGHT_CONTROLLER_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <tiltrotor_drone/flight_controllerConfig.h>

#include <gazebo_msgs/ModelStates.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <cmath>

//#include <mavros_msgs/DebugValue.h>
//#include <gazebo_msgs/ContactsState.h>

class FlightController {
private:
    // Publishers
    ros::Publisher _current_drone_pose_pub,
                   _current_drone_twist_pub,
                   _setpoint_raw_pub,
                   //_servo_angle_pub,
                   //_contact_force_pub,
                   _send_controller_pub,
                   _tilt_pub; 

    // Subscribers
    ros::Subscriber _gazebo_model_state_sub,
                    _current_drone_state_sub;
                    //_contact_force_sub;

    // Clients
    ros::ServiceClient _arm_cli,
                       _set_mode_cli;

    // Published messages
    geometry_msgs::PoseStamped _current_drone_pose_msg;
    geometry_msgs::TwistStamped _current_drone_twist_msg;
    mavros_msgs::AttitudeTarget _setpoint_raw_msg;
    //std_msgs::Float64 _servo_angle_msg;
    //std_msgs::Float64 _contact_force_msg;
    std_msgs::Float64 _tilt_msg; 

    // Subscribed messages
    gazebo_msgs::ModelStates _gazebo_model_state_msg;
    mavros_msgs::State _current_drone_state_msg;
    //gazebo_msgs::ContactsState _contact_force_input_msg;

    // Service messages
    mavros_msgs::CommandLong _arm_msg;
    mavros_msgs::SetMode _mode_msg;
    tiltrotor_drone::flight_controllerConfig last_config_;
    tiltrotor_drone::flight_controllerConfig config_;

    // Variables
    std::string _drone_name;
    bool _drone_name_received;
    int _index;
    double dt;
    double dt1;
    double dt2;
    //float _contact_f;
    //double _x_sp_temp;
    //double _y_sp_temp;
    //double _z_sp_temp;
    bool _sp_adjusted;
    struct {double x,y,x_local,y_local,vx_local,vy_local,z,yaw_deg,yaw_rad,pitch_deg,pitch_rad,roll_deg,roll_rad,vx,vy,vz,q;} _current;
    struct {double x,y,z,yaw_deg,pitch_deg,servo_angle,x_new,y_new;} _desired;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz,q;} _e;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _i_e;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _d_e;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _previous_e;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _kp;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _ki;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _kd;
    struct {double x,z;} _f;
    struct {double x,y,z,roll,pitch,yaw,vx,vy,vz;} _u;
    struct {double x,y,z;} _lim_u;
    double _tilt; 
    double _T;  
    double _m;
    bool _yaw_flag;
    std::vector<double> _E;

    std::chrono::high_resolution_clock::time_point _t0, _t1;
    std::chrono::duration<double> _dt;
    std::chrono::high_resolution_clock::time_point _t2, _t3;
    std::chrono::duration<double> _dt_1;
    std::chrono::high_resolution_clock::time_point _t4, _t5;
    std::chrono::duration<double> _dt_2;

public:
    // Constructor
    FlightController(){};

    // Destructor
    ~FlightController(){};

    // Variables
    double node_rate;

    // Callback functions
    void cbReconfig(tiltrotor_drone::flight_controllerConfig &config, uint32_t level);
    void cbGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void cbDroneState(const mavros_msgs::State::ConstPtr &msg);
    //void cbContactsState(const gazebo_msgs::ContactsState::ConstPtr &msg);

    // Other functions
    dynamic_reconfigure::Server <tiltrotor_drone::flight_controllerConfig> reconfig_ser;

    void init(int argc, char** argv);
    void initRosComms(ros::NodeHandle *nh);
    void initParams();
    void sendCurrentDronePose();
    void sendCmds();

    double getYawDeg(geometry_msgs::PoseStamped &msg);
    double getPitchDeg(geometry_msgs::PoseStamped &msg);
    double getRollDeg(geometry_msgs::PoseStamped &msg);

};
#endif
