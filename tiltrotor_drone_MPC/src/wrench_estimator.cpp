#include <wrench_estimator.hpp>

// Constants
const Eigen::Vector3d e3(0, 0, 1);
const Eigen::MatrixXd tol = 0.001 * Eigen::MatrixXd::Identity(6, 6);
Eigen::Matrix3d Zero3() {
    return Eigen::Matrix3d::Zero();
}

// Rotation matrix from Euler angles (body frame to inertial frame)
Eigen::Matrix3d rotationMatrixFromEuler(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R_x, R_y, R_z;

    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll),  cos(roll);

    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
          -sin(pitch), 0, cos(pitch);

    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
           0, 0, 1;

    return R_z * R_y * R_x;
}

// Transformation matrix (Euler rates to body angular velocity)
Eigen::Matrix3d TMatrix(double pitch, double roll) {
    Eigen::Matrix3d T_matrix;

    T_matrix << 1, 0, -sin(pitch),
         0, cos(roll), sin(roll)*cos(pitch),
         0, -sin(roll), cos(roll)*cos(pitch);

    return T_matrix;
}

// Gravity vector in body frame
Eigen::VectorXd G_function(double m, double g, const Eigen::Vector3d& states) {
    double roll = states[0];
    double pitch = states[1];
    double yaw = states[2];

    Eigen::Matrix3d R = rotationMatrixFromEuler(roll, pitch, yaw);
    Eigen::Vector3d gravity_inertial(0, 0, m * g);
    Eigen::Vector3d gravity_body = R.transpose() * gravity_inertial;

    Eigen::VectorXd G(6);
    G << gravity_body, 0.0, 0.0, 0.0;
    return G;
}

// H matrix
Eigen::MatrixXd H_matrix(double m, const Eigen::Matrix3d& J, const Eigen::Matrix3d& Q) {
    Eigen::Matrix3d M1 = m * Eigen::Matrix3d::Identity();
    //Eigen::Matrix3d M2 = Q.transpose() * J * Q; // Inversion causes numerical instability
    Eigen::Matrix3d M2 = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd H(6, 6);
    H << M1, Zero3(),
         Zero3(), M2;
    return H;
}

// C matrix
Eigen::MatrixXd C_matrix(
    const Eigen::Matrix3d& Q,
    const Eigen::Matrix3d& Qdot,
    const Eigen::Vector3d& rates,
    const Eigen::Matrix3d& J
) {
    Eigen::Vector3d S0 = Q * rates;
    Eigen::Matrix3d S;
    S <<     0, -S0(2),  S0(1),
          S0(2),     0, -S0(0),
         -S0(1),  S0(0),     0;
    Eigen::Matrix3d V = Q.transpose() * S * J * Q + Q.transpose() * J * Qdot;

    Eigen::MatrixXd C(6, 6);
    C << Zero3(), Zero3(),
         Zero3(), V;
    return C;
}

// Torques
Eigen::Vector3d torques_vector(const Eigen::VectorXd& rotors_thrust_pmw, double tilt, double kf, double Lh, double Lv, double m, double g) {
    double cos_tilt = cos(-tilt);
    double sin_tilt = sin(-tilt);
    kf = 1 / kf;
    
    Eigen::MatrixXd D(3, 4);
    D << -Lh*cos_tilt-kf*sin_tilt, Lh*cos_tilt+kf*sin_tilt, Lh*cos_tilt-kf*sin_tilt, -Lh*cos_tilt+kf*sin_tilt,
         -Lv*cos_tilt, -Lv*cos_tilt, Lv*cos_tilt, Lv*cos_tilt,
         Lh*sin_tilt-kf*cos_tilt, -Lh*sin_tilt+kf*cos_tilt, -Lh*sin_tilt-kf*cos_tilt, Lh*sin_tilt+kf*cos_tilt;
         
    Eigen::VectorXd rotors_thrust_newton = ((rotors_thrust_pmw.array() - 1000) / 1000.0) * (m * g) / 0.612 / 4;          
    Eigen::Vector3d torques = D * rotors_thrust_newton;   

    return torques;
}

WRENCHEstimator::WRENCHEstimator() {
    _t0 = std::chrono::high_resolution_clock::now();
    _t1 = _t0;
    Q_prev = Eigen::Matrix3d::Identity();
    gamma = Eigen::VectorXd::Zero(6);
    W_f = Eigen::VectorXd::Zero(6);
}

// Main Wrench computation
std::pair<Eigen::VectorXd, double> WRENCHEstimator::computeWrench(
    const Eigen::Vector3d& attitude,
    const Eigen::Vector3d& velocity,
    Eigen::Vector3d& rates,
    double T,
    const Eigen::VectorXd& rotors_thrust,
    double tilt,
    double m,
    double a,
    double tau,
    double I_x,
    double I_y,
    double I_z,
    double g,
    double kf,
    double Lv,
    double Lh
) {
    // Physical parameters 
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
    J(0, 0) = I_x;
    J(1, 1) = I_y;
    J(2, 2) = I_z;

    // Rotation and transformation matrices
    double roll = attitude[0], pitch = attitude[1], yaw = attitude[2];
    Eigen::Matrix3d R = rotationMatrixFromEuler(roll, pitch, yaw);
    Eigen::Matrix3d T_mat = TMatrix(pitch, roll);
    Eigen::Matrix3d Q = T_mat;

    // Compute time delta
    _t1 = std::chrono::high_resolution_clock::now();
    dt = std::max(std::chrono::duration<double>(_t1 - _t0).count(), 0.001);

    // Compute Qdot using previous Q
    Eigen::Matrix3d Qdot = (Q - Q_prev) / dt;
    Q_prev = Q;

    // System dynamics
    Eigen::VectorXd G = G_function(m, g, attitude);
    Eigen::MatrixXd H = H_matrix(m, J, Q);
    Eigen::MatrixXd C = C_matrix(Q, Qdot, rates, J);
    
    // First derivatives vector in body frame
    Eigen::Vector3d velocity_body = R.transpose() * velocity;
    Eigen::VectorXd vel_rates(6);
    vel_rates << velocity_body, rates;
    
    // Thrust in body frame
    Eigen::Matrix3d R_tilt = rotationMatrixFromEuler(0, -tilt, 0);
    Eigen::Vector3d T_tilt = (R_tilt * e3) * T;
    
    // Torques in body frame
    Eigen::Vector3d torques = torques_vector(rotors_thrust, tilt, kf, Lh, Lv, m, g);
    Eigen::VectorXd u(6);
    u << T_tilt, torques;
    
    // Estimation of wrench
    Eigen::VectorXd B = -gamma + C * vel_rates - a * vel_rates + G - u;
    Eigen::VectorXd I = ((H + tol)).ldlt().solve(B); 
    gamma = gamma + a * I * dt;
    _t0 = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd W = gamma + a * vel_rates;

    // Exponential filter
    double alpha = tau / (tau + dt);
    W_f = alpha * W_f + (1 - alpha) * W;
    
    return std::make_pair(W_f, torques[1]); // Second output is to debug
}
