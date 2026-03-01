#pragma once
#include"../A_include.h"

class PID{
public:
    PID();
    double P;
    double I;
    double D;
    Matrix<double,6,1>Eta_up;
    Matrix<double,6,1>err_sum;
    Matrix<double,6,1> Pid(Matrix<double,6,1>& target,
                       vector<Matrix<double,6,1>>& Eta_list);
};


