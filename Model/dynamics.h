#pragma once
#include"../A_include.h"

//输入力，输出速度
Matrix<double, 6, 1>dynamics(Matrix<double, 6, 6>& M_,Matrix<double, 6, 6>& C_V,
                             Matrix<double, 6, 6>& D_V,Matrix<double, 6, 1>& g_Eta,
                             Matrix<double, 6, 1>& Tau,double tau_e,Matrix<double, 6, 1>& V);
