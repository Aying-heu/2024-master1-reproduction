#include"Control.h"

MPC mpc1;
S_VUD_FLC s_vud_flc1;
MatrixXd Controller_trace_to_Force(double t,
        double T,
        double t_max,
        const Eigen::Matrix<double, 6, 1>& current_Eta,
        const Eigen::Matrix<double, 6, 1>& current_V,
        const vector<Eigen::Matrix<double, 5, 1>>& reference_Eta){

    MatrixXd DesiredVelocity;
    DesiredVelocity=mpc1.ComputeDesiredVelocity(t,T,t_max,current_Eta,current_V,reference_Eta);

    Matrix<double,6,1> DesiredForce;
    DesiredForce=s_vud_flc1.ComputeDesiredForce(T,current_V,DesiredVelocity);

    Matrix<double,8,1> distribution;
    distribution=Thrust_distribution(DesiredForce);

    MatrixXd Force_Generated;
    Force_Generated=Generate_Force(distribution);


cout<<"2"<<endl;
cout<<"t=               \t"<<t<<endl;
cout<<"current_V=       \t"<<current_V.transpose()<<endl;
cout<<"current_Eta=     \t"<<current_Eta.transpose()<<endl;
cout<<"reference_Eta=   \t"<<reference_Eta[std::floor(t/T)+1].transpose()<<endl;
cout<<"DesiredVelocity= \t"<<DesiredVelocity.transpose()<<endl;
cout<<"DesiredForce=    \t"<<DesiredForce.transpose()<<endl;
cout<<"Force_Generated= \t"<<Force_Generated.transpose()<<endl;
cout<<"/***********************************************************************************************************************";


    return Force_Generated;
}
