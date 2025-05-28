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

// Initialize the parameters
void FlightController::initRosComms(ros::NodeHandle *nh)
{
    // Publishers
    _setpoint_raw_pub =
    nh->advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);

    _current_drone_pose_pub =
    nh->advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
    
    _current_drone_twist_pub =
    nh->advertise<geometry_msgs::TwistStamped>("mavros/vision_pose/velocity", 1);
    
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

    //_contact_force_sub = 
    //nh->subscribe<gazebo_msgs::ContactsState>("contactsensor",
    //                                  1,
    //                                  &FlightController::cbContactsState,
    //                                  this);

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
    ros::param::get(ros::this_node::getName() + "/kp_x", _kp.x);
    ros::param::get(ros::this_node::getName() + "/ki_x", _ki.x);
    ros::param::get(ros::this_node::getName() + "/kd_x", _kd.x);
    ros::param::get(ros::this_node::getName() + "/kp_y", _kp.y);
    ros::param::get(ros::this_node::getName() + "/ki_y", _ki.y);
    ros::param::get(ros::this_node::getName() + "/kd_y", _kd.y);
    ros::param::get(ros::this_node::getName() + "/kp_z", _kp.z);
    ros::param::get(ros::this_node::getName() + "/ki_z", _ki.z);
    ros::param::get(ros::this_node::getName() + "/kd_z", _kd.z);
    ros::param::get(ros::this_node::getName() + "/kp_roll", _kp.roll);
    ros::param::get(ros::this_node::getName() + "/ki_roll", _ki.roll);
    ros::param::get(ros::this_node::getName() + "/kd_roll", _kd.roll);
    ros::param::get(ros::this_node::getName() + "/kp_yaw", _kp.yaw);
    ros::param::get(ros::this_node::getName() + "/ki_yaw", _ki.yaw);
    ros::param::get(ros::this_node::getName() + "/kd_yaw", _kd.yaw);
    ros::param::get(ros::this_node::getName() + "/kp_vy", _kp.vy);
    ros::param::get(ros::this_node::getName() + "/ki_vy", _ki.vy);
    ros::param::get(ros::this_node::getName() + "/kd_vy", _kd.vy);
    ros::param::get(ros::this_node::getName() + "/lim_ux", _lim_u.x);
    ros::param::get(ros::this_node::getName() + "/lim_uy", _lim_u.y);
    ros::param::get(ros::this_node::getName() + "/lim_uz", _lim_u.z);
    ros::param::get(ros::this_node::getName() + "/kp_tau", _kp.tau);
    ros::param::get(ros::this_node::getName() + "/ki_tau", _ki.tau);
    ros::param::get(ros::this_node::getName() + "/kd_tau", _kd.tau);
    ros::param::get(ros::this_node::getName() + "/m", _m);
    ros::param::get(ros::this_node::getName() + "/I", _I);
    ros::param::get(ros::this_node::getName() + "/g", _g);

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
        
        _current.p_global = _current_drone_twist_msg.twist.angular.x;
        _current.q_global = _current_drone_twist_msg.twist.angular.y;
        _current.r_global = _current_drone_twist_msg.twist.angular.z;
        std::array<double, 3> _current_rates_global = {_current.p_global, _current.q_global, _current.r_global};
        
        std::array<std::array<double, 3>, 3> R_matrix = Inertial2BodyRotationMatrix(_current.roll_rad, _current.pitch_rad, _current.yaw_rad);
        std::array<double, 3> _current_rates_body = multiplyMatrixVector(R_matrix, _current_rates_global);
        
        _old_q = _current.q;
        _current.q = _current_rates_body[1];
        
        _current.vx = _current_drone_twist_msg.twist.linear.x;
        _current.vy = _current_drone_twist_msg.twist.linear.y;
        _current.vz = _current_drone_twist_msg.twist.linear.z;
        
        K = getK();
        
        //_current.f = _contact_f;
        
        double cos_yaw = cos(_current.yaw_rad - last_config_.desired_yaw*3.1416/180);
        double sin_yaw = sin(_current.yaw_rad - last_config_.desired_yaw*3.1416/180);
    
        if (abs(_desired.yaw_deg - last_config_.desired_yaw) > 0.1) {
            _yaw_flag = false;  
        }
    
        if (!_yaw_flag) { 
            _desired.x_new1 = _desired.x * cos_yaw + _desired.y * sin_yaw;
            _desired.y_new1 = -_desired.x * sin_yaw + _desired.y * cos_yaw;    
            if (abs(_current.yaw_deg - _desired.yaw_deg) < 0.5) {
                _yaw_flag = true;
                last_config_.desired_x = _desired.x_new1;
                _desired.x = _desired.x_new1;
                last_config_.desired_y = _desired.y_new1;
                _desired.y = _desired.y_new1;
                last_config_.desired_z = _desired.z;
                last_config_.desired_pitch = _desired.pitch_deg;
                last_config_.desired_yaw = _desired.yaw_deg;
                reconfig_ser.updateConfig(last_config_);            
            }    
        } else {
            _desired.x_new1 = _desired.x;
            _desired.y_new1 = _desired.y;      
        }
       
        std::array<std::array<double, 3>, 3> R_matrix_new = Inertial2BodyRotationMatrix(_current.roll_rad, _current.pitch_rad, 0);
        std::array<double, 3> _current_position_global = {_current.x, _current.y, _current.z};
        std::array<double, 3> _current_position_body = multiplyMatrixVector(R_matrix, _current_position_global);
        std::array<double, 3> _current_velocity_global = {_current.vx, _current.vy, _current.vz};
        std::array<double, 3> _current_velocity_body = multiplyMatrixVector(R_matrix, _current_velocity_global);
        std::array<double, 3> _desired_position_global = {_desired.x_new1, _desired.y_new1, _desired.z};
        std::array<double, 3> _desired_position_body = multiplyMatrixVector(R_matrix_new, _desired_position_global);
        
        _current.x_local = _current_position_body[0];
        _current.y_local = _current_position_body[1];
        _current.z_local = _current_position_body[2];
       
        _current.vx_local = _current_velocity_body[0];
        _current.vy_local = _current_velocity_body[1];
        _current.vz_local = _current_velocity_body[2]; 
       
        _desired.x_new = _desired_position_body[0];
        _desired.y_new = _desired_position_body[1];
        _desired.z_new = _desired_position_body[2]; 
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
    
    // Calculate errors and changes in errors to be input to PID controllers
    _t1 = std::chrono::high_resolution_clock::now();
    _dt = std::chrono::duration_cast<std::chrono::duration<double>>(_t1 - _t0);

    dt = std::max(_dt.count(), 0.001);
    
    if (std::isnan(dt) || std::isinf(dt)) {
        ROS_ERROR("Invalid duration detected.");
        return;
    }
      
    // Velocity and y-direction controller
    _previous_e.x = _e.x;
    _previous_e.y = _e.y;
    _previous_e.z = _e.z;
    //_previous_e.f = _e.f;
    
    _e.x = _desired.x_new - _current.x_local;
    _e.y = _desired.y_new - _current.y_local;
    _e.z = _desired.z_new - _current.z_local;
    //_e.f = _current.f - _desired.f;
    
    _i_e.x += _e.x * dt;
    _i_e.y += _e.y * dt;
    _i_e.z += _e.z * dt;
    //_i_e.f += _e.f * dt;
    
    _d_e.x = (_e.x - _previous_e.x) / dt;
    _d_e.y = (_e.y - _previous_e.y) / dt;
    _d_e.z = (_e.z - _previous_e.z) / dt;
    //_d_e.f = (_e.f - _previous_e.f) / dt;
    
    _t0 = std::chrono::high_resolution_clock::now();
    
    _u.x = _kp.x * _e.x + _kd.x * _d_e.x + _ki.x * _i_e.x;
    _u.y = _kp.y * _e.y + _kd.y * _d_e.y + _ki.y * _i_e.y;
    _u.z = _kp.z * _e.z + _kd.z * _d_e.z + _ki.z * _i_e.z;
    
    _u.x = std::min(_u.x, _lim_u.x);
    _u.x = std::max(_u.x, -_lim_u.x);

    _u.y = std::min(_u.y, _lim_u.y);
    _u.y = std::max(_u.y, -_lim_u.y);
    
    _u.z = std::min(_u.z, _lim_u.z);
    _u.z = std::max(_u.z, -_lim_u.z);
    
    _t3 = std::chrono::high_resolution_clock::now();
    _dt_1 = std::chrono::duration_cast<std::chrono::duration<double>>(_t3 - _t2);

    dt1 = std::max(_dt_1.count(), 0.001);
    
    if (std::isnan(dt1) || std::isinf(dt1)) {
        ROS_ERROR("Invalid duration detected.");
        return;  // Handle error
    }
    
    _previous_e.vy = _e.vy;
    _e.vy = _u.y - _current.vy_local;  
    _i_e.vy += _e.vy * dt1;
    _d_e.vy = (_e.vy - _previous_e.vy) / dt1;
    
    _t2 = std::chrono::high_resolution_clock::now();
     
    _u.vy = -(_kp.vy * _e.vy + _kd.vy * _d_e.vy + _ki.vy * _i_e.vy);
    
    //ROS_INFO("sp roll: %f", _u.vy);
    //ROS_INFO("current roll: %f", _current.roll_deg);
    
    // Yaw, Roll and Pitch controller
    _t5 = std::chrono::high_resolution_clock::now();
    _dt_2 = std::chrono::duration_cast<std::chrono::duration<double>>(_t5 - _t4);

    dt2 = std::max(_dt_2.count(), 0.001);
    
    if (std::isnan(dt2) || std::isinf(dt2)) {
        ROS_ERROR("Invalid duration detected.");
        return;
    }
    
    _previous_e.yaw = _e.yaw;
    _previous_e.roll = _e.roll;
        
    _e.yaw =  _desired.yaw_deg * 3.1416 / 180 - _current.yaw_rad;
    _e.pitch =  _desired.pitch_deg * 3.1416 / 180 - _current.pitch_rad;
    _e.roll = _u.vy * 3.1416 / 180 - _current.roll_rad;
            
    _i_e.yaw += _e.yaw * dt2;
    _i_e.roll += _e.roll * dt2;
    
    _d_e.yaw = (_e.yaw - _previous_e.yaw) / dt2;
    _d_e.roll = (_e.roll - _previous_e.roll) / dt2; 

    _t4 = std::chrono::high_resolution_clock::now();
    
    _u.yaw = _kp.yaw * _e.yaw + _kd.yaw * _d_e.yaw + _ki.yaw * _i_e.yaw;
    _u.roll = _kp.roll * _e.roll + _kd.roll * _d_e.roll + _ki.roll * _i_e.roll;
   
    // LQR controller
    _e.vx = _u.x - _current.vx_local;
    _e.vz = _u.z - _current.vz_local;
    _e.q = 0 - _current.q;
    
    std::vector<double> _E = {_e.x, _e.vx, _e.z, _e.vz, _e.pitch, _e.q}; 
    std::vector<double> LQR(3, 0.0);
    std::vector<std::vector<double>> K = getK();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            LQR[i] += K[i][j] * _E[j];
        }
    }
      
    double cos_pitch = cos(_current.pitch_rad);
    double sin_pitch = sin(_current.pitch_rad);
    
    _f.x = LQR[0] - _m*_g*sin_pitch;
    _f.z = LQR[1] + _m*_g*cos_pitch;
    _tilt = -atan(_f.x / _f.z);
    _T = (sqrt(pow(_f.x, 2) + pow(_f.z, 2)))/(2*_g*_m*0.817);
    
    _t7 = std::chrono::high_resolution_clock::now();
    _dt_3 = std::chrono::duration_cast<std::chrono::duration<double>>(_t7 - _t6);

    dt3 = std::max(_dt_3.count(), 0.001);
    
    if (std::isnan(dt3) || std::isinf(dt3)) {
        ROS_ERROR("Invalid duration detected.");
        return;
    }
    
    _current.tau = (_current.q - _old_q) / dt3 * _I;    
    _previous_e.tau = _e.tau;     
    _e.tau = LQR[2] - _current.tau;      
    _i_e.tau += _e.tau * dt3;
    _d_e.tau = (_e.tau - _previous_e.tau) / dt3;

    _t6 = std::chrono::high_resolution_clock::now();
    
    _u.tau = _kp.tau * _e.tau + _kd.tau * _d_e.tau + _ki.tau * _i_e.tau;
    
    _setpoint_raw_msg.type_mask = 128;   
    _setpoint_raw_msg.body_rate.x = _u.roll;
    _setpoint_raw_msg.body_rate.y = _u.tau;
    _setpoint_raw_msg.body_rate.z = _u.yaw;
    _setpoint_raw_msg.thrust = _T;
    _setpoint_raw_pub.publish(_setpoint_raw_msg);
    
    _tilt_msg.data = _tilt;       
    _tilt_pub.publish(_tilt_msg); 
    
    ROS_INFO("sp tilt: %f", _tilt*180/3.1416);
    ROS_INFO("thrust: %f", _T);
    ROS_INFO("fx: %f", LQR[0]);
    ROS_INFO("fz: %f", LQR[1]);

    //_servo_angle_msg.data = 3.1416 * (_desired.servo_angle / 180);
    //_servo_angle_pub.publish(_servo_angle_msg);

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

std::vector<std::vector<double>> FlightController::getK()
{
    std::vector<std::vector<double>> K(3, std::vector<double>(6, 0.0));
    
    K[0][0] = 5;
    K[0][1] = 8.756;
    K[1][2] = 5;
    K[1][3] = 8.756;
    K[2][4] = 5;
    K[2][5] = 5.147;
    
    return K;
}

std::array<std::array<double, 3>, 3> FlightController::Inertial2BodyRotationMatrix(double roll, double pitch, double yaw) {
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    std::array<std::array<double, 3>, 3> R_bi = {{       
        {cy * cp,                        sy * cp,                  -sp},
        {cy * sp * sr - sy * cr,         sy * sp * sr + cy * cr,   cp * sr},
        {cy * sp * cr + sy * sr,         sy * sp * cr - cy * sr,   cp * cr}
    }};

    return R_bi;
}

std::array<double, 3> FlightController::multiplyMatrixVector(const std::array<std::array<double, 3>, 3>& matrix, const std::array<double, 3>& vec) {
    std::array<double, 3> result = {0.0, 0.0, 0.0};

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            result[i] += matrix[i][j] * vec[j];
        }
    }

    return result;
}

