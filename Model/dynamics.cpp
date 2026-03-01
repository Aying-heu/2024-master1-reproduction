#include"dynamics.h"

Matrix<double, 6, 1>dynamics(Matrix<double, 6, 6>& M_,Matrix<double, 6, 6>& C_V,
                             Matrix<double, 6, 6>& D_V,Matrix<double, 6, 1>& g_Eta,
                             Matrix<double, 6, 1>& Tau,double tau_e,Matrix<double, 6, 1>& V){

    Matrix<double, 6, 1>V_dot;
    Matrix<double, 6, 1> tau_E;

    tau_E<<tau_e,tau_e,tau_e,tau_e,tau_e,tau_e;

    V_dot=M_.inverse()*(Tau+tau_E-g_Eta-D_V*V-C_V*V);
    return V_dot;
 }
