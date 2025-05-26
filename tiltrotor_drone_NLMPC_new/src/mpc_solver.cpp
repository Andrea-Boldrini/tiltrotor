#include <mpc_solver.hpp>
#include <ros/ros.h>
#include <flight_controller.h>
#include <qpOASES.hpp>
using namespace casadi;

/// ----------------- ROTATION MATRIX -----------------
SX rotationMatrixFromEuler(const SX& roll, const SX& pitch, const SX& yaw) {
    SX R_x = SX::eye(3);
    R_x(1, 1) = cos(roll); R_x(1, 2) = -sin(roll);
    R_x(2, 1) = sin(roll); R_x(2, 2) = cos(roll);
                      
    SX R_y = SX::eye(3);
    R_y(0,0) = cos(pitch); R_y(0,2) = sin(pitch);
    R_y(2,0) = -sin(pitch); R_y(2,2) = cos(pitch);

    SX R_z = SX::eye(3);
    R_z(0, 0) = cos(yaw); R_z(0, 1) = -sin(yaw);
    R_z(1, 0) = sin(yaw); R_z(1, 1) = cos(yaw);

    return SX::mtimes({R_z, R_y, R_x});
}

SX T_matrix_inverse(const SX& phi, const SX& theta) {
    SX T_inv = SX::eye(3);
    
    double limit = 75 * 3.1416 / 180.0;
    SX clamp_theta = if_else(theta < -limit, -limit,
                      if_else(theta > limit, limit, theta));
    
    SX sin_phi = sin(phi);
    SX cos_phi = cos(phi);
    SX tan_theta = tan(clamp_theta);
    SX cos_theta = cos(clamp_theta);

    T_inv(0,0) = 1;
    T_inv(0,1) = sin_phi * tan_theta;
    T_inv(0,2) = cos_phi * tan_theta;

    T_inv(1,0) = 0;
    T_inv(1,1) = cos_phi;
    T_inv(1,2) = -sin_phi;

    T_inv(2,0) = 0;
    T_inv(2,1) = sin_phi / cos_theta;
    T_inv(2,2) = cos_phi / cos_theta;

    return T_inv;
}

/// ----------------- MPC SOLVER CONSTRUCTOR -----------------
MPCSolver::MPCSolver() {
    ros::NodeHandle nh;
    nx_ = 12;
    nu_ = 5;
    nd_ = 6;
    
    // Load parameters
    nh.param<int>("/flight_controller/Np", Np_, 15);
    nh.param<int>("/flight_controller/Nc", Nc_, 5);
    nh.param<double>("/flight_controller/Ts", Ts_, 0.1);
    nh.param<bool>("/flight_controller/_grasp_flag", grasp_enabled, false);

    // Physical parameters
    mass_ = 5.1664;
    g_ = 9.8066;
    I_x_ = 0.026;
    I_y_ = 0.150;
    I_z_ = 0.146;
    SX I = SX::diag(SX::vertcat({I_x_, I_y_, I_z_}));
    SX p_gripper_body = SX::vertcat({0.27, 0, -0.035});
    SX p_tree = SX::vertcat({1, 1, 0});

    // Define symbolic variables
    SX x = SX::sym("x", nx_);
    SX u = SX::sym("u", nu_);
    SX d = SX::sym("d", nd_);

    // State update equations
    SX x_dot = SX::zeros(nx_);
    
    x_dot(0) = x(1);
    x_dot(2) = x(3);
    x_dot(4) = x(5);
    SX rates = x(Slice(7, nx_, 2));
    
    SX T_inv = T_matrix_inverse(x(6), x(8));
    SX euler_derivative = SX::mtimes(T_inv,rates);
    x_dot(6) = euler_derivative(0);
    x_dot(8) = euler_derivative(1);
    x_dot(10) = euler_derivative(2);

    // Dynamics equations
    SX R = rotationMatrixFromEuler(x(6), x(8), 0);
    SX dis = d(Slice(0, 3));
    SX forces = SX::vertcat({
    -mass_ * g_ * cos(x(6)) * sin(x(8)),
    0,
    mass_ * g_ * cos(x(6)) * cos(x(8))
    });
    SX F = (1.0 / mass_) * (SX::mtimes(R, (SX::vertcat({u(0), 0, u(1)}) + forces + dis)) - g_ * mass_ * SX::vertcat({0, 0, 1}));
    x_dot(Slice(1, nx_/2, 2)) = F;

    // Angular dynamics
    SX torque = u(Slice(2, 5));
    x_dot(Slice(7, nx_, 2)) = SX::mtimes(SX::inv(I), (torque + d(Slice(3, 6)) - SX::cross(rates, SX::mtimes(I, rates))));
    
    // Next state calculation
    SX x_next = x + Ts_ * x_dot;

    // Define dynamics as a CasADi function
    dynamics_ = Function("dynamics", {x, u, d}, {x_next});

    /// ----------------- DEFINE COST AND CONSTRAINTS -----------------
    SX P = SX::sym("P", 2 * nx_ + Np_ * nd_);
    SX x0 = P(Slice(0, nx_));
    SX x_ref = P(Slice(nx_, 2 * nx_));

    std::vector<SX> U, D;
    for (int i = 0; i < Nc_; ++i) U.push_back(SX::sym("u_" + std::to_string(i), nu_));
    for (int i = 0; i < Np_; ++i) D.push_back(P(Slice(2 * nx_ + i * nd_, 2 * nx_ + (i + 1) * nd_)));

    SX cost = 0;
    SX Q_cost = SX::diag(SX::vertcat({8, 3, 3, 1, 8, 3, 1, 0.5, 1.5, 1, 1.5, 1}));
    SX R_cost = SX::diag(SX::vertcat({0.01, 0.01, 0.01, 0.01, 0.05}));
    SX X = x0;
    std::vector<SX> g;
    std::vector<double> lbg_vec;
    std::vector<double> ubg_vec;
    for (int i = 0; i < Np_; ++i) {
        SX u = (i < Nc_) ? U[i] : U.back();
        SX d_i = D[i];
        X = dynamics_(std::vector<SX>{X, u, d_i})[0];
        SX err = X - x_ref;
        cost += SX::mtimes(SX::mtimes(err.T(), Q_cost), err) + SX::mtimes(SX::mtimes(u.T(), R_cost), u);
        
        g.push_back(X(6));
        lbg_vec.push_back(-30*3.1416/180);
        ubg_vec.push_back(30*3.1416/180);
        
        g.push_back(X(8)); 
        lbg_vec.push_back(-1.57);
        ubg_vec.push_back(1.57);
        
        //g.push_back(X(0));
        //lbg_vec.push_back(-0.2);
        //ubg_vec.push_back(0.2);
        
        //g.push_back(X(4));
        //lbg_vec.push_back(0);
        //ubg_vec.push_back(1.2);
        
        if (grasp_enabled) {
            SX p_uav_vehicle = SX::vertcat({X(0), X(2), X(4)});
            SX R_i = rotationMatrixFromEuler(X(6), X(8), 0);
            SX p_gripper_vehicle = p_uav_vehicle + SX::mtimes(R_i, p_gripper_body);
            SX err_grasp = p_gripper_vehicle - p_tree;
            for (int j = 0; j < 3; ++j) {
                g.push_back(err_grasp(j));
                lbg_vec.push_back(0.0);
                ubg_vec.push_back(0.0);
            }
        }
    }
    
    SX g_all = vertcat(g);
    SX U_flat = vertcat(U);
    lbx_ = DM::repmat(DM::vertcat({-20, -40, -16, -16, -8}), Nc_, 1);
    ubx_ = DM::repmat(DM::vertcat({20, 40, 16, 16, 8}), Nc_, 1);
    lbg_ = DM(lbg_vec);
    ubg_ = DM(ubg_vec);

    // Solver definition
    SXDict nlp = {{"x", U_flat}, {"f", cost}, {"p", P}, {"g", g_all}};
    Dict opts;
    opts["qpsol"] = "qpoases";    
    opts["qpsol_options.printLevel"] = "none"; 
    opts["print_time"] = false;           
    opts["verbose"] = false;               
    opts["print_header"] = false;           
    solver_ = nlpsol("solver", "sqpmethod", nlp, opts);

    p_cache_ = DM::zeros(2 * nx_ + Np_ * nd_);
    previous_u_opt_ = DM::zeros(nu_ * Nc_);
    
    arg_["x0"] = previous_u_opt_;
    arg_["p"] = p_cache_;
    arg_["lbx"] = lbx_;
    arg_["ubx"] = ubx_;
    arg_["lbg"] = lbg_;
    arg_["ubg"] = ubg_;
}

std::pair<std::vector<double>, double> MPCSolver::solve(const std::vector<double>& x_vec,
                                                        const std::vector<double>& x_ref_vec,
                                                        const std::vector<double>& d_seq) {
    // Step 1: Populate the parameter cache (P vector)
    for (int i = 0; i < nx_; ++i) {
        p_cache_(i) = x_vec[i];
        p_cache_(nx_ + i) = x_ref_vec[i];
    }

    for (int i = 0; i < d_seq.size(); ++i) {
        p_cache_(2 * nx_ + i) = d_seq[i];
    }

    // Step 2: Define solver arguments
    arg_["x0"] = previous_u_opt_;  
    arg_["p"] = p_cache_;

    // Step 3: Solve the optimization problem
    auto sol = solver_(arg_);

    // Step 4: Extract the optimal control
    DM u0 = sol.at("x")(Slice(0, nu_));

    // Step 5: Update the cache for warm start
    previous_u_opt_ = sol.at("x");

    // Step 6: Return the first control action and the time step
    return {std::vector<double>(u0->begin(), u0->end()), Ts_}; // Second output is for debug
}

