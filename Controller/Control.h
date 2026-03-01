#pragma once


#include"OuterLoop_MPC.h"
#include"InnerLoop_S-VUD FLC.h"
#include"Thrust_Allocation.h"

MatrixXd Controller_trace_to_Force(double t,
        double T,
        double t_max,
        const Eigen::Matrix<double, 6, 1>& current_Eta,
        const Eigen::Matrix<double, 6, 1>& current_V,
        const vector<Eigen::Matrix<double, 5, 1>>& reference_Eta);
