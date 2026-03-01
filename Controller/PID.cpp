#include"PID.h"

PID::PID():
    P(50),I(5),D(2){

    err_sum<<0,0,0,0,0,0;
    Eta_up<<0,0,0,0,0,0;
}


Matrix<double,6,1> PID::Pid(Matrix<double,6,1>& target,
                       vector<Matrix<double,6,1>>& Eta_list){
    Matrix<double,6,1>output;

    Matrix<double,6,1> err=target-Eta_list.back();
    err_sum+=err;
    err_sum(0)=err_sum(0)<-100?-100:err_sum(0);
    err_sum(1)=err_sum(1)<-100?-100:err_sum(1);
    err_sum(2)=err_sum(2)<-200?-200:err_sum(2);
    err_sum(3)=err_sum(3)<-PI?-PI:err_sum(3);
    err_sum(4)=err_sum(4)<-PI?-PI:err_sum(4);
    err_sum(5)=err_sum(5)<-PI?-PI:err_sum(5);

    err_sum(0)=err_sum(0)>100?100:err_sum(0);
    err_sum(1)=err_sum(1)>100?100:err_sum(1);
    err_sum(2)=err_sum(2)>200?200:err_sum(2);
    err_sum(3)=err_sum(3)>PI?PI:err_sum(3);
    err_sum(4)=err_sum(4)>PI?PI:err_sum(4);
    err_sum(5)=err_sum(5)>PI?PI:err_sum(5);

    output=P*err+err_sum*I+D*(Eta_list.back()-Eta_up);
    Eta_up=Eta_list.back();

    return output;
}
