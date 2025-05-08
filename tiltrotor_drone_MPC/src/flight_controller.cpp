/**
 * package:  tiltrotor_drone
 * node:     flight_controller
 * file:     flight_controller.cpp
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

// Callback functions
void FlightController::cbReconfig(tiltrotor_drone::flight_controllerConfig &config, uint32_t level)
{
    _desired.x = config.desired_x;
    _desired.y = config.desired_y;
    _desired.z = config.desired_z;   
    last_config_.desired_yaw = _desired.yaw_deg;
    _desired.yaw_deg = config.desired_yaw;
    _desired.pitch_deg = config.desired_pitch;
    //_desired.servo_angle = config.desired_servo_angle;
    //_kp.f = config.kp_f;
    //_ki.f = config.ki_f;
    //_kd.f = config.kd_f;
    //_desired.f = config.desired_f;    
}

void FlightController::cbGazeboModelState(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int i = 0;
    while (!_drone_name_received)
    {
        if (!msg->name[i].compare(_drone_name))
        {
            _index = i;
            _drone_name_received = true;
        }
        i++;
    }

    if (_drone_name_received)
    {
        _current_drone_pose_msg.pose.position.x = msg->pose[_index].position.x;
        _current_drone_pose_msg.pose.position.y = msg->pose[_index].position.y;
        _current_drone_pose_msg.pose.position.z = msg->pose[_index].position.z;
        
        _current_drone_twist_msg.twist.linear.x = msg->twist[_index].linear.x;
        _current_drone_twist_msg.twist.linear.y = msg->twist[_index].linear.y;
        _current_drone_twist_msg.twist.linear.z = msg->twist[_index].linear.z;
        
        _current_drone_twist_msg.twist.angular.x = msg->twist[_index].angular.x;
        _current_drone_twist_msg.twist.angular.y = msg->twist[_index].angular.y;
        _current_drone_twist_msg.twist.angular.z = msg->twist[_index].angular.z;

        _current_drone_pose_msg.pose.orientation.x = msg->pose[_index].orientation.x;
        _current_drone_pose_msg.pose.orientation.y = msg->pose[_index].orientation.y;
        _current_drone_pose_msg.pose.orientation.z = msg->pose[_index].orientation.z;
        _current_drone_pose_msg.pose.orientation.w = msg->pose[_index].orientation.w;
    }
}

void FlightController::cbDroneState(const mavros_msgs::State::ConstPtr &msg)
{
    _current_drone_state_msg = *msg;
}

//void FlightController::cbContactsState(const gazebo_msgs::ContactsState::ConstPtr &msg)
//{
//    if (!msg->states.empty()) 
//    {
//        _contact_f = msg->states[0].total_wrench.force.x; 
//        _contact_force_msg.data = msg->states[0].total_wrench.force.x; 
//    } 
//   else 
//    {
//        _contact_f = 0.0;
//        _contact_force_msg.data = 0.0;
//   }
//    ROS_INFO("contact force: %f", _contact_f);
//    _contact_force_pub.publish(_contact_force_msg);
//}

void FlightController::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    _commanded_wrench_msg = *msg;
    _wrench_published = true;
    _dis_state.clear();
    for (int i = 0; i < 15; ++i) {
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.force.x);
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.force.y);
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.force.z*0.57);
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.torque.x);
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.torque.y);
        _dis_state.insert(_dis_state.end(), _commanded_wrench_msg.wrench.torque.z*3);
    }
}

void FlightController::actuatorCallback(const mavros_msgs::RCOut::ConstPtr &msg)
{
    _actuator_msg = *msg;
    _CoG_pos = 0.13 * (_actuator_msg.channels[0] - _actuator_msg.channels[1] ) / (_actuator_msg.channels[0] + _actuator_msg.channels[1]); 
}

// Initialize the parameters
void FlightController::initRosComms(ros::NodeHandle *nh)
{
    // Publishers
    _setpoint_raw_pub =
    nh->advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);

    _current_drone_pose_pub =
    nh->advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1);
    
    _current_drone_twist_pub =
    nh->advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1);
    
    _tilt_pub = nh->advertise<std_msgs::Float64>("tiltrotor/angle_setpoint", 1);    
    
    // _servo_angle_pub = 
    // nh->advertise<std_msgs::Float64>("joint_position_cmd", 1);

    //_servo_angle_pub = //no cable setup
    //nh->advertise<std_msgs::Float64>("gripper_joint_position_cmd", 1);

    //_contact_force_pub = 
    //nh->advertise<std_msgs::Float64>("force_value", 1);

    // Subscribers
    _gazebo_model_state_sub =
    nh->subscribe<gazebo_msgs::ModelStates>("gazebo/model_states",
                                            1,
                                            &FlightController::cbGazeboModelState,
                                            this);
    _current_drone_state_sub =
    nh->subscribe<mavros_msgs::State>("mavros/state",
                                      1,
                                      &FlightController::cbDroneState,
                                      this);
                                      
    _commanded_wrench_sub = 
    nh->subscribe<geometry_msgs::WrenchStamped>("/applied_wrenches", 
                  1,
                  &FlightController::wrenchCallback, 
                  this);

    //_contact_force_sub = 
    //nh->subscribe<gazebo_msgs::ContactsState>("contactsensor",
    //                                  1,
    //                                  &FlightController::cbContactsState,
    //                                  this);
    
    _actuator_sub = nh->subscribe<mavros_msgs::RCOut>("mavros/rc/out", 1, &FlightController::actuatorCallback, this);

    // Servers
    dynamic_reconfigure::Server<tiltrotor_drone::flight_controllerConfig>::CallbackType reconfig_f;
    reconfig_f = boost::bind(&FlightController::cbReconfig, this, _1, _2);
    reconfig_ser.setCallback(reconfig_f);

    // Clients
    _arm_cli =
    nh->serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    _set_mode_cli =
    nh->serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void FlightController::initParams()
{
    ros::param::get(ros::this_node::getName() + "/node_rate", node_rate);
    ros::param::get(ros::this_node::getName() + "/drone_name", _drone_name);
    ros::param::get(ros::this_node::getName() + "/kp_tau_x", _kp.tau_x);
    ros::param::get(ros::this_node::getName() + "/ki_tau_x", _ki.tau_x);
    ros::param::get(ros::this_node::getName() + "/kd_tau_x", _kd.tau_x);
    ros::param::get(ros::this_node::getName() + "/kp_tau_y", _kp.tau_y);
    ros::param::get(ros::this_node::getName() + "/ki_tau_y", _ki.tau_y);
    ros::param::get(ros::this_node::getName() + "/kd_tau_y", _kd.tau_y);
    ros::param::get(ros::this_node::getName() + "/kp_tau_z", _kp.tau_z);
    ros::param::get(ros::this_node::getName() + "/ki_tau_z", _ki.tau_z);
    ros::param::get(ros::this_node::getName() + "/kd_tau_z", _kd.tau_z);
    ros::param::get(ros::this_node::getName() + "/m", _m);
    ros::param::get(ros::this_node::getName() + "/I_x", _I.x);
    ros::param::get(ros::this_node::getName() + "/I_y", _I.y);
    ros::param::get(ros::this_node::getName() + "/I_z", _I.z);
    ros::param::get(ros::this_node::getName() + "/g", _g);
    ros::param::get(ros::this_node::getName() + "/a", _a);
    ros::param::get(ros::this_node::getName() + "/tau_filter", _tau_filter);
    ros::param::get(ros::this_node::getName() + "/Np", _Np);
    ros::param::get(ros::this_node::getName() + "/Nc", _Nc);
    ros::param::get(ros::this_node::getName() + "/Ts", _Ts);
    ros::param::get(ros::this_node::getName() + "/kf", _kf);
    ros::param::get(ros::this_node::getName() + "/Lh", _Lh);
    ros::param::get(ros::this_node::getName() + "/Lv", _Lv);

    _drone_name_received = false;
    _sp_adjusted = false;
}

void FlightController::sendCurrentDronePose()
{
    if (_drone_name_received)
    {
        _current_drone_pose_msg.header.stamp = ros::Time::now();
        _current_drone_pose_msg.header.frame_id = "map";
        _current_drone_pose_pub.publish(_current_drone_pose_msg);

        _current.x = _current_drone_pose_msg.pose.position.x;
        _current.y = _current_drone_pose_msg.pose.position.y;
        _current.z = _current_drone_pose_msg.pose.position.z;
        
        _current.yaw_deg = getYawDeg(_current_drone_pose_msg);        
        if ((_current.yaw_deg < 0 ) && (_desired.yaw_deg > 160)) {
        _current.yaw_rad = 3.1416 * ((_current.yaw_deg + 360) / 180);
        } else if ((_current.yaw_deg > 0 ) && (_desired.yaw_deg < -160)) {
        _current.yaw_rad = 3.1416 * ((_current.yaw_deg - 360) / 180);
        } else {
        _current.yaw_rad =  3.1416 * (_current.yaw_deg / 180);
        }        
        _current.pitch_deg = getPitchDeg(_current_drone_pose_msg);
        _current.pitch_rad =  3.1416 * (_current.pitch_deg / 180);
        _current.roll_deg = getRollDeg(_current_drone_pose_msg);
        _current.roll_rad =  3.1416 * (_current.roll_deg / 180);
        
        _old.p = _current.p;
        _current.p = _current_drone_twist_msg.twist.angular.x;
        _old.q = _current.q;
        _current.q = _current_drone_twist_msg.twist.angular.y;
        _old.r = _current.r;
        _current.r = _current_drone_twist_msg.twist.angular.z;
        
        _current.vx = _current_drone_twist_msg.twist.linear.x;
        _current.vy = _current_drone_twist_msg.twist.linear.y;
        _current.vz = _current_drone_twist_msg.twist.linear.z;
        
        //_current.f = _contact_f;
        
        double cos_yaw = cos(_current.yaw_rad);
        double sin_yaw = sin(_current.yaw_rad);
        
       _current.x_local = _current.x * cos_yaw + _current.y * sin_yaw;
       _current.y_local = -_current.x * sin_yaw + _current.y * cos_yaw;
       _current.vx_local = _current.vx * cos_yaw + _current.vy * sin_yaw;
       _current.vy_local = -_current.vx * sin_yaw + _current.vy * cos_yaw;
       
    }
}

void FlightController::sendCmds()
{
    if (_current_drone_state_msg.mode != "OFFBOARD")
    {
        _mode_msg.request.custom_mode = "OFFBOARD";
        _set_mode_cli.call(_mode_msg);
    }

    if (!_current_drone_state_msg.armed)
    {
        _arm_msg.request.broadcast = false;
        _arm_msg.request.command = 400;
        _arm_msg.request.confirmation = 0;
        _arm_msg.request.param1 = 1;
        _arm_msg.request.param2 = 21196;
        _arm_cli.call(_arm_msg);
    }

    //if (_desired.servo_angle >= 90)
    //{
    //    //current xyz = setpoint xyz
    //    if (_sp_adjusted == false)
    //    {
    //        _x_sp_temp = _current.x;
    //        _y_sp_temp = _current.y;
    //        _z_sp_temp = _current.z;
    //        _sp_adjusted = true;
    //    }
    //    _desired.x = _x_sp_temp;
    //    _desired.y = _y_sp_temp;
    //    _desired.z = _z_sp_temp;
    //}
    //else 
    //{
    //   _sp_adjusted = false;
    //}
    
    double cos_yaw = cos(_current.yaw_rad - last_config_.desired_yaw*3.1416/180);
    double sin_yaw = sin(_current.yaw_rad - last_config_.desired_yaw*3.1416/180);
    
    if (abs(_desired.yaw_deg - last_config_.desired_yaw) > 0.1) {
        _yaw_flag = false;    
    }
    
    if (!_yaw_flag) {
        _desired.x_new = _desired.x * cos_yaw + _desired.y * sin_yaw;
        _desired.y_new = -_desired.x * sin_yaw + _desired.y * cos_yaw;        
        if (abs(_current.yaw_deg - _desired.yaw_deg) < 0.5) {
            _yaw_flag = true;
            last_config_.desired_x = _desired.x_new;
            _desired.x = _desired.x_new;
            last_config_.desired_y = _desired.y_new;
            _desired.y = _desired.y_new;
            last_config_.desired_z = _desired.z;
            last_config_.desired_pitch = _desired.pitch_deg;
            last_config_.desired_yaw = _desired.yaw_deg;
            reconfig_ser.updateConfig(last_config_);            
       }          
    } else {
       _desired.x_new = _desired.x;
       _desired.y_new = _desired.y;
    }
   
    // MPC controller and wrench estimator
    std::vector<double> current_state(12);
    std::vector<double> ref_state(12);
    
    current_state[0] = _current.x_local;
    current_state[1] = _current.vx_local;
    current_state[2] = _current.y_local;
    current_state[3] = _current.vy_local;
    current_state[4] = _current.z;
    current_state[5] = _current.vz;
    current_state[6] = _current.roll_rad;
    current_state[7] = _current.p;
    current_state[8] = _current.pitch_rad;
    current_state[9] = _current.q;
    current_state[10] = _current.yaw_rad;
    current_state[11] = _current.r;
    
    ref_state[0] = _desired.x_new;
    ref_state[1] = 0;
    ref_state[2] = _desired.y_new;
    ref_state[3] = 0;
    ref_state[4] = _desired.z;
    ref_state[5] = 0;
    ref_state[6] = 0;
    ref_state[7] = 0;
    ref_state[8] = _desired.pitch_deg*3.1416/180;
    ref_state[9] = 0;
    ref_state[10] = _desired.yaw_deg*3.1416/180;
    ref_state[11] = 0;
    
    std::vector<double> _attitude(3);
    std::vector<double> _rates(3);
    std::vector<double> _velocity(3);
    std::vector<double> _rotors_thrust(4);
    
    _attitude[0] = _current.roll_rad;
    _attitude[1] = _current.pitch_rad;
    _attitude[2] = _current.yaw_rad;
    _rates[0] = _current.p;
    _rates[1] = _current.q;
    _rates[2] = _current.r;
    _velocity[0] = _current.vx;
    _velocity[1] = _current.vy;
    _velocity[2] = _current.vz;
    if (!_actuator_msg.channels.empty()) {
        _rotors_thrust[0] = _actuator_msg.channels[0]; //front
        _rotors_thrust[1] = _actuator_msg.channels[2]; //front
        _rotors_thrust[2] = _actuator_msg.channels[1]; //rear
        _rotors_thrust[3] = _actuator_msg.channels[3]; //rear
    } else {
       _rotors_thrust[0] = 0;
       _rotors_thrust[1] = 0;
       _rotors_thrust[2] = 0;
       _rotors_thrust[3] = 0;
    }
    
    Eigen::Vector3d attitude(_attitude[0], _attitude[1], _attitude[2]);
    Eigen::Vector3d velocity(_velocity[0], _velocity[1], _velocity[2]);
    Eigen::Vector3d rates(_rates[0], _rates[1], _rates[2]);
    Eigen::Vector4d rotors_thrust(_rotors_thrust[0], _rotors_thrust[1], _rotors_thrust[2], _rotors_thrust[3]);
    
    //_wrench_published = false;
    if (!_wrench_published) {
        _dis_state = std::vector<double>(6 * _Np, 0.0);
    }   
    std::pair<Eigen::VectorXd, double> dis = wrench_estimator.computeWrench(attitude, velocity, rates, _T_tot, rotors_thrust, _tilt, _m, _a, _tau_filter, _I.x, _I.y, _I.z, _g, _kf, _Lv, _Lh);
    Eigen::VectorXd W_f = dis.first;
    Eigen::VectorXd W_pred = W_f;
    double debug = dis.second;

    if (!_wrench_published) {    
       W_f[0] = 0;
       W_f[1] = 0;
       W_f[2] = 0;
       W_f[3] = 0;
       W_f[4] = 0;
       W_f[5] = 0;
    } else {
       W_f[3] = 0;
       W_f[4] = 0;
       W_f[5] = 0;
    }

    std::vector<double> _dis_state_est;
    for (int i = 0; i < _Np; ++i) {
        _dis_state_est.insert(_dis_state_est.end(), W_f.data(), W_f.data() + W_f.size());
        //a_dis_state_est.insert(_dis_state_est.end(), _dis_state.data(), _dis_state.data() + _dis_state.size());
    }
    MPC = mpc.solve(current_state, ref_state, _dis_state_est);

    double cos_pitch = cos(_current.pitch_rad);
    double sin_pitch = sin(_current.pitch_rad);
    double cos_roll = cos(_current.roll_rad);
    
    _f.x = MPC[0] - _m*_g*sin_pitch*cos_roll;
    _f.z = MPC[1] + _m*_g*cos_pitch*cos_roll;
    _tilt = -atan(_f.x / _f.z);
    _T = (sqrt(pow(_f.x, 2) + pow(_f.z, 2)))/(2*_g*_m*0.817);
    _T_tot = sqrt(pow(_f.x, 2) + pow(_f.z, 2));
    
    _t7 = std::chrono::high_resolution_clock::now();
    _dt_3 = std::chrono::duration_cast<std::chrono::duration<double>>(_t7 - _t6);

    dt3 = std::max(_dt_3.count(), 0.001);
    
    if (std::isnan(dt3) || std::isinf(dt3)) {
        ROS_ERROR("Invalid duration detected.");
        return;
    }
    
    _current.tau_x = (_current.p - _old.p) / dt3 * _I.x;
    _previous_e.tau_x = _e.tau_x;   
    _e.tau_x = MPC[2] - _current.tau_x;       
    _i_e.tau_x += _e.tau_x * dt3;
    _d_e.tau_x = (_e.tau_x - _previous_e.tau_x) / dt3;
    
    _current.tau_y = (_current.q - _old.q) / dt3 * _I.y;
    _previous_e.tau_y = _e.tau_y;   
    _e.tau_y = MPC[3] + W_f[4] - _current.tau_y;       
    _i_e.tau_y += _e.tau_y * dt3;
    _d_e.tau_y = (_e.tau_y - _previous_e.tau_y) / dt3;
    
    _current.tau_z = (_current.r - _old.r) / dt3 * _I.z;
    _previous_e.tau_z = _e.tau_z;   
    _e.tau_z = MPC[4] - _current.tau_z;       
    _i_e.tau_z += _e.tau_z * dt3;
    _d_e.tau_z = (_e.tau_z - _previous_e.tau_z) / dt3;
    
    _t6 = std::chrono::high_resolution_clock::now();
    
    _u.tau_x = _kp.tau_x * _e.tau_x + _kd.tau_x * _d_e.tau_x + _ki.tau_x * _i_e.tau_x;
    _u.tau_y = _kp.tau_y * _e.tau_y + _kd.tau_y * _d_e.tau_y + _ki.tau_y * _i_e.tau_y;
    _u.tau_z = _kp.tau_z * _e.tau_z + _kd.tau_z * _d_e.tau_z + _ki.tau_z * _i_e.tau_z;
    
    _setpoint_raw_msg.type_mask = 128;

    _setpoint_raw_msg.body_rate.x = _u.tau_x;
    _setpoint_raw_msg.body_rate.y = _u.tau_y;
    _setpoint_raw_msg.body_rate.z = _u.tau_z;
    _setpoint_raw_msg.thrust = _T;
    _setpoint_raw_pub.publish(_setpoint_raw_msg);
    
    _tilt_msg.data = _tilt;       
    _tilt_pub.publish(_tilt_msg); 
    
    //ROS_INFO("sp tilt: %f", _tilt*180/3.1416);
    //ROS_INFO("thrust: %f", _T_tot);
    //ROS_INFO("thrust [0-1]: %f", _T);
    //ROS_INFO("fx_MPC: %f", MPC[0]);
    //ROS_INFO("fz_MPC: %f", MPC[1]);
    ROS_INFO("tau_x_MPC: %f", MPC[2]);
    ROS_INFO("tau_y_MPC: %f", MPC[3]);
    ROS_INFO("tau_z_MPC: %f", MPC[4]);
    
    ROS_INFO("Fx_est: %f", W_pred[0]);
    ROS_INFO("Fy_est: %f", W_pred[1]);
    ROS_INFO("Fz_est: %f", W_pred[2]);
    ROS_INFO("tau_x_est: %f", W_pred[3]);
    ROS_INFO("tau_y_est: %f", W_pred[4]);
    ROS_INFO("tau_z_est: %f", W_pred[5]);
    
    //ROS_INFO("Fx_true: %f", _dis_state[0]);
    //ROS_INFO("Fy_true: %f", _dis_state[1]);
    //ROS_INFO("Fz_true: %f", _dis_state[2]);
    //ROS_INFO("tau_x_true: %f", _dis_state[3]);
    //ROS_INFO("tau_y_true: %f", _dis_state[4]);
    //ROS_INFO("tau_z_true: %f", _dis_state[5]);
    
    //ROS_INFO("sp x: %f", _desired.x);
    ROS_INFO("current x: %f", _current.x_local);
    //ROS_INFO("sp y: %f", _desired.y);
    ROS_INFO("current y: %f", _current.y_local);
    //ROS_INFO("sp z: %f", _desired.z);
    ROS_INFO("current z: %f", _current.z);
    //ROS_INFO("sp pitch: %f", _desired.pitch_deg);
    ROS_INFO("current pitch: %f", _current.pitch_deg);
    //ROS_INFO("sp yaw: %f", _desired.yaw_deg);
    ROS_INFO("current yaw: %f", _current.yaw_deg);
    ROS_INFO("current roll: %f", _current.roll_deg);

    //_servo_angle_msg.data = 3.1416 * (_desired.servo_angle / 180);
    //_servo_angle_pub.publish(_servo_angle_msg);
    
    //if (!_actuator_msg.channels.empty()) {
    //    ROS_INFO("PMW1: %u", _actuator_msg.channels[0]);
    //    ROS_INFO("PMW3: %u", _actuator_msg.channels[1]);
    //    ROS_INFO("PMW2: %u", _actuator_msg.channels[2]);
    //    ROS_INFO("PMW4: %u", _actuator_msg.channels[3]);
    //    ROS_INFO("x: %f", _CoG_pos);
    //} else {
    //    ROS_WARN("No data in _actuator_msg.channels!");
    //}
    ROS_INFO("debug: %f", debug);

}

double FlightController::getYawDeg(geometry_msgs::PoseStamped &msg)
{
    tf2::Quaternion q(msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = (yaw / 3.1416) * 180;

    return yaw;
}

double FlightController::getPitchDeg(geometry_msgs::PoseStamped &msg)
{
    tf2::Quaternion q(msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pitch = (pitch / 3.1416) * 180;

    return pitch;
}

double FlightController::getRollDeg(geometry_msgs::PoseStamped &msg)
{
    tf2::Quaternion q(msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll = (roll / 3.1416) * 180;

    return roll;
}
