#pragma once
#include"../A_include.h"

class PARAMS{
public:
PARAMS();
    void updateV_u() ;
    void updateEta_x();
    void update_Tau_X();
    void update_J();
    void update_CV();
    void update_DV();
    void update_g_Eta();

    void update_position_list();
    void update_pose_list();
    void update_speed_list();
    void update_angular_speed_list();
    void update_pose();

    void Update_Params();

    void Calculate(int mode);

    double m;
    double u, v, w, p, q, r;
    double x, y, z, phi, theta, psi;
    double X,Y,Z,K,M,N;
    Matrix<double, 6, 1>target;
    Matrix<double, 6, 1> V;
    Matrix<double, 6, 1> V_dot;
    Matrix<double, 6, 1> Eta;
    Matrix<double, 6, 1> Eta_dot;
    vector<double> Times;
    vector<Matrix<double, 6, 1>>Eta_list;
    vector<Matrix<double, 6, 1>>Tau_list;
    vector<Matrix<double, 5, 1>>Reference_Eta;

    Matrix<double, 3, 3> zeroMatrix;
    Matrix<double, 3, 3> Rx;
    Matrix<double, 3, 3> Ry;
    Matrix<double, 3, 3> Rz;
    Matrix<double, 3, 3> J1_eta2;
    Matrix<double, 3, 3> J2_eta2;
    Matrix<double, 6, 6> J_eta;

    Matrix<double, 6, 6> M_RB;
    Matrix<double, 6, 6> M_A;
    Matrix<double, 6, 6> M_;
    Matrix<double, 6, 6> CA_V;
    Matrix<double, 6, 6> CRB_V;
    Matrix<double, 6, 6> C_V;
    Matrix<double, 6, 6> DL;
    Matrix<double, 6, 6> DQ_absV;
    Matrix<double, 6, 6> D_V;
    double W, B;
    double xg,yg,xb,yb,zg,zb;
    Matrix<double, 6, 1> g_Eta;

    double t;
    double T;
    double t_max;
    vector<double> tau_E;
    Matrix<double, 6, 1> Tau;

    Matrix<double, 3, 1> list_init;
    vector<Matrix<double, 3, 1>> position_list;
    vector<Matrix<double, 3, 1>> pose_list;
    vector<Matrix<double, 3, 1>> speed_list;
    vector<Matrix<double, 3, 1>> angular_speed_list;

    Isometry3d pose = Isometry3d::Identity();
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;


    void Write_data_v(vector<double>& data,string filename);
    void Write_data_vv(vector<Matrix<double,5,1>>& data, const std::string& filename);
    void Write_data_vv(vector<Matrix<double,6,1>>& data, const std::string& filename);

};







