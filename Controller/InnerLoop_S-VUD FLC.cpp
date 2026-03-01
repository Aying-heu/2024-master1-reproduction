// S_VUD_FLC.cpp
#include "InnerLoop_S-VUD FLC.h"
#include <iostream>

S_VUD_FLC::S_VUD_FLC():
gamma(0.85),lambda(0.5),c(0.1),Ki(100),p{0.5,0.5}
{

    err_up << 0, 0, 0, 0, 0, 0;
    err_dot << 0, 0, 0, 0, 0, 0;
    x2_sum<<0,0,0,0,0,0;
}

Eigen::Matrix<double, 6, 1>S_VUD_FLC::ComputeDesiredForce(
        double T,
        const Eigen::Matrix<double, 6, 1>& actual_velocity,
        const Eigen::Matrix<double, 6, 1>& desired_velocity){

    Matrix<double, 6, 1> DesiredForce;

    err=desired_velocity-actual_velocity;
    err_dot=(err-err_up)/T;
    x2_sum+=err_dot;

    for(int i=0;i<actual_velocity.rows();i++){
        double ys;
        double x1,x2;
        x1=err(i);
        x2=err_dot(i);
        ys=fs_x(x1/alpha_x(x1),x2/alpha_x(x2))/beta_x(x1,x2)+Ki*x2_sum(i);
        DesiredForce(i)=ys;
    }

    err_up=err;
    return DesiredForce;
}

double S_VUD_FLC::alpha_x(double x){
    return 1-gamma*exp(-0.5*x*x);
}
double S_VUD_FLC::fs_x(double x1,double x2){
    return lambda*tanh(c*x1)+(1-lambda)*tanh(c*x2);
}
double S_VUD_FLC::beta_x(double x1,double x2){
    double beta=p[0]*x1+p[1]*x2;
    if (std::abs(beta) < 1e-6) {  // 检查 beta 是否接近零
        beta = 1e-6;  // 设置一个最小值
    }
    return beta;
}
/*
MatrixXd S_VUD_FLC::Mexp(Matrix A){

    MatrixXd result(A.rows(),A.cols());
    for(int i=0;i<a.rows();i++)
        for(int j=0;i<A.cols();j++)
            result(i,j)=exp(A(i,j));
    return result;
}*/
