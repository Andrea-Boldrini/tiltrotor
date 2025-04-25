#ifndef WRENCH_ESTIMATOR_HPP
#define WRENCH_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <chrono>
#include <utility> 

class WRENCHEstimator{
public:
    WRENCHEstimator();

    std::pair<Eigen::VectorXd, double> computeWrench(
        const Eigen::Vector3d& attitude,
        const Eigen::Vector3d& velocity,
        Eigen::Vector3d& rates,
        double T,
        const Eigen::Vector3d& torques,
        double tilt
    );

private:
    double dt;
    std::chrono::high_resolution_clock::time_point _t0, _t1;
    
    Eigen::Matrix3d Q_prev;
    Eigen::VectorXd gamma;   
    Eigen::VectorXd W_f;       
};


#endif
