// ThrustAllocation.h
#pragma once
#include "../A_include.h"
/*
class ThrustAllocation {
public:
    ThrustAllocation();
    std::vector<double> AllocateThrust(
        const Eigen::Matrix<double, 6, 1>& control_input);

private:
    // 推力分配矩阵
    Eigen::Matrix<double, 6, 8> thrust_allocation_matrix;
};
*/

Matrix<double,8,1>Thrust_distribution(Matrix<double,6,1>& DesiredVelocity);
Matrix<double,6,1>Generate_Force(Matrix<double,8,1>&Thrust_distribution);
