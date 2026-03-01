// ThrustAllocation.cpp
#include "Thrust_Allocation.h"
#include <iostream>

Matrix<double,8,1>Thrust_distribution(Matrix<double,6,1>& DesiredVelocity){

    double X,Y,Z,K,M,N;
    X=DesiredVelocity(0);
    Y=DesiredVelocity(1);
    Z=DesiredVelocity(2);
    K=DesiredVelocity(3);
    M=DesiredVelocity(4);
    N=DesiredVelocity(5);

    Matrix<double,8,1>distribution;
    distribution<<  X+Y+N,
                    X-Y+N,
                    X-Y+N,
                    X+Y-N,
                    Z-K-M,
                    Z+K-M,
                    Z-K+M,
                    Z+K+M;
    return distribution;
}
UUV_MODEL uuv0;
Matrix<double,6,1>Generate_Force(Matrix<double,8,1>&Thrust_distribution){
    Matrix<double,6,1> Force_Generated;
    Force_Generated=uuv0.B*Thrust_distribution;
    return Force_Generated;
}

/*
ThrustAllocation::ThrustAllocation() {
    // 初始化推力分配矩阵
   // thrust_allocation_matrix <<  根据AUV的推进器布局填充矩阵
}

std::vector<double> ThrustAllocation::AllocateThrust(
    const Eigen::Matrix<double, 6, 1>& control_input) {
    // 实现推力分配算法
    // 返回每个推进器的推力指令
    std::vector<double> thrust_commands(8);
    // ...
    return thrust_commands;
}
*/
