// S_VUD_FLC.h
#pragma once
#include "../A_include.h"

class S_VUD_FLC {
public:
    S_VUD_FLC();
    Eigen::Matrix<double, 6, 1> ComputeDesiredForce(
        double T,
        const Eigen::Matrix<double, 6, 1>& actual_velocity,
        const Eigen::Matrix<double, 6, 1>& desired_velocity);

    // S-VUD FLC参数

    double gamma;
    double p[2];
    double lambda;
    double c;
    double Ki;

    Matrix<double, 6, 1> err;
    Matrix<double, 6, 1> err_up;
    Matrix<double, 6, 1> err_dot;
    Matrix<double, 6, 1> x2_sum;

    double alpha_x(double x);
    double fs_x(double x1,double x2);
    double beta_x(double x1,double x2);
};
