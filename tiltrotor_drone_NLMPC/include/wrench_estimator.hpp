#ifndef WRENCH_ESTIMATOR_HPP
#define WRENCH_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <chrono>
#include <utility> 

class WRENCHEstimator{
public:
    WRENCHEstimator();

    std::pair<Eigen::VectorXd, Eigen::Vector3d> computeWrench(
        const Eigen::Vector3d& attitude,
        const Eigen::Vector3d& velocity,
        Eigen::Vector3d& rates_global,
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
    );

private:
    double dt;
    std::chrono::high_resolution_clock::time_point _t0, _t1;
    
    Eigen::VectorXd gamma;   
    Eigen::VectorXd W_f;  
    Eigen::VectorXd W_f_body;       
};


#endif
