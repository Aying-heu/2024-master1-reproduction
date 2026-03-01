#include"kinematics.h"

Matrix<double, 6, 1> kinematics(Matrix<double, 6, 6>& J_eta,Matrix<double, 6, 1>& V){
    Matrix<double, 6, 1> Eta_dot;
    Eta_dot=J_eta*V;
    return Eta_dot;
}
