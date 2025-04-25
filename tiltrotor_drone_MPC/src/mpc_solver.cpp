#include <mpc_solver.hpp>
#include <casadi/casadi.hpp>
#include <qpOASES.hpp>

using namespace casadi;

MPCSolver::MPCSolver() {
    nx_ = 12;
    nu_ = 5;
    nd_ = 6;
    Np_ = 15;
    Nc_ = 3;
    Ts_ = 0.1;

    mass_ = 5.2;
    g_ = 9.81;
    I_x_ = 0.026;
    I_y_ = 0.150;
    I_z_ = 0.146;

    // Continuous A, B
    DM A = DM::zeros(nx_, nx_);
    A(0,1) = 1;
    A(2,3) = 1;
    A(3,6) = -g_;
    A(4,5) = 1;
    A(6,7) = 1;
    A(8,9) = 1;
    A(10,11) = 1;

    DM B = DM::zeros(nx_, nu_);
    B(1,0) = 1 / mass_;
    B(5,1) = 1 / mass_;
    B(7,2) = 1 / I_x_;
    B(9,3) = 1 / I_y_;
    B(11,4) = 1 / I_z_;
    
    DM B_d = DM::zeros(nx_, nd_);
    B_d(1,0) = 1 / mass_;
    B_d(3,1) = 1 / mass_;
    B_d(5,2) = 1 / mass_;
    B_d(7,3) = 1 / I_x_;
    B_d(9,4) = 1 / I_y_;
    B_d(11,5) = 1 / I_z_;
    
    DM B_aug = horzcat(B, B_d);

    // Discretize using forward Euler
    DM Ad = DM::eye(nx_) + Ts_ * A;
    DM Bd = Ts_ * B_aug;
    
    DM B_input_ = Bd(Slice(), Slice(0, nu_));
    DM B_disturb_ = Bd(Slice(), Slice(nu_, nu_ + nd_));
    
    //std::cout << "Ad = " << Ad << std::endl;
    //std::cout << "Bd = " << Bd << std::endl;

    // Variables
    MX P = MX::sym("P", 2 * nx_ + Np_ * nd_);
    MX x = P(Slice(0, nx_));
    MX x_ref = P(Slice(nx_, 2 * nx_));
    std::vector<MX> U;
    for (int i = 0; i < Nc_; ++i) { 
        U.push_back(MX::sym("u_" + std::to_string(i), nu_));
    }
    std::vector<MX> D;
    for (int i = 0; i < Np_; ++i) {
        D.push_back(P(Slice(2 * nx_ + i * nd_, 2 * nx_ + (i + 1) * nd_)));
    }

    // Cost function
    DM Q = DM::diag(DM::vertcat({
    7.0,    // x
    1.0,    // vx
    1.0,    // y
    1.0,    // vy
    5.0,    // z
    1.0,    // vz
    2,    // roll
    0,    // p
    2.0,    // pitch
    1.0,    // q
    3.0,    // yaw
    1.0    // r
    }));

    DM R = DM::diag(DM({
    0.1,    // fx
    0.1,    // fz
    0.01,    // tau x
    0.01,    // tau y
    0.01,    // tau z
    }));

    MX X = x;
    MX cost = MX::zeros();
    std::vector<MX> g;
    std::vector<DM> lbg_vec;
    std::vector<DM> ubg_vec;
    for (int i = 0; i < Np_; ++i) {
        MX u;
        if (i < Nc_) {
             u = U[i];
        } else {
             u = U[Nc_ - 1];
        }

        MX d = D[i];
        X = mtimes(Ad, X) + mtimes(B_input_, u) + mtimes(B_disturb_, d);
        MX e = X - x_ref;
        cost += (mtimes(mtimes(e.T(), Q), e) + mtimes(mtimes(u.T(), R), u));
        
        g.push_back(X(6));
        lbg_vec.push_back(-3.1416/180*10);
        ubg_vec.push_back(3.1416/180*10);
    }

    MX g_all = vertcat(g);
    MX U_flat = vertcat(U);
    
    // Input constraints
    DM u_min = DM::vertcat({-10.0, -20.0, -8.0, -8.0, -4.0});
    DM u_max = DM::vertcat({ 10.0,  20.0,  8.0,  8.0,  4.0});
    lbx_ = DM::repmat(u_min, Nc_, 1); 
    ubx_ = DM::repmat(u_max, Nc_, 1); 
    lbg_ = vertcat(lbg_vec);
    ubg_ = vertcat(ubg_vec);

    MXDict nlp = {{"x", U_flat}, {"f", cost}, {"p", P}, {"g", g_all}};
    solver_ = nlpsol("solver", "sqpmethod", nlp);
    
    p_cache_ = DM::zeros(2 * nx_ + Np_ * nd_);
    previous_u_opt_ = DM::zeros(nu_ * Nc_);

    arg_["x0"] = previous_u_opt_;
    arg_["p"] = p_cache_;
    arg_["lbx"] = lbx_;
    arg_["ubx"] = ubx_;
    arg_["lbg"] = lbg_;
    arg_["ubg"] = ubg_;

}

std::vector<double> MPCSolver::solve(const std::vector<double>& x_vec, const std::vector<double>& x_ref_vec, const std::vector<double>& d_seq) {

    for (int i = 0; i < nx_; ++i)
        p_cache_(i) = x_vec[i];

    for (int i = 0; i < nx_; ++i)
        p_cache_(nx_ + i) = x_ref_vec[i];

    for (int i = 0; i < Np_; ++i) {
        for (int j = 0; j < nd_; ++j) {
            int idx = 2 * nx_ + i * nd_ + j;
            int d_idx = i * nd_ + j;
            p_cache_(idx) = d_seq[d_idx];
        }
    }
    
    arg_["x0"] = previous_u_opt_;
    arg_["p"] = p_cache_;

    auto sol = solver_(arg_);
    //auto stats = solver_.stats();
    //std::string status = stats.at("return_status");
    //std::cout << "Solver status: " << status << std::endl;
    DM u0 = sol["x"](Slice(0, nu_));
    std::vector<double> u_opt(u0->begin(), u0->end());
    
    previous_u_opt_ = sol["x"](casadi::Slice(0, nu_ * Nc_));

    return u_opt; // [Fx, Fz, τx, τy, τz]
}

