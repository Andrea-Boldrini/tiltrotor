#pragma once

#include <casadi/casadi.hpp>
#include <vector>

class MPCSolver {
public:
    MPCSolver();
    std::vector<double> solve(const std::vector<double>& x, const std::vector<double>& x_ref,const std::vector<double>& d_seq);

private:
    casadi::DM lbx_, ubx_;
    casadi::DM lbg_, ubg_;
    casadi::Function solver_;
    casadi::DMDict arg_;
    casadi::DM p_cache_;
    casadi::DM previous_u_opt_;
    double mass_, g_, I_x_, I_y_, I_z_, Ts_;
    int Np_, nx_, nu_, nd_, Nc_;
};

