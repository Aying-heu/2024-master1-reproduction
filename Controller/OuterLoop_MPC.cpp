// MPC.cpp
#include "OuterLoop_MPC.h"


MPC::MPC():
    // 初始化MPC参数
    prediction_horizon_Np(20),
    control_horizon_Nc(10){
    Q = MatrixXd::Identity(5, 5) * 100;  // 输出信号的加权矩阵
    R = MatrixXd::Identity(5, 5) * 0.1;  // 控制信号的加权矩阵

    V_max<< 0.7, 0.4, 0.7, 1.25, 1.25;
    V_min<< -0.7, -0.4, -0.7, -1.25, -1.25;
    U_max<< 0.08, 0.08, 0.08, 0.15, 0.15;
    U_min<< -0.08, -0.08, -0.08, -0.15, -0.15;

    I2=Matrix<double,5,5>::Identity();
    O=Matrix<double, 5, 5>::Zero();
    I=Matrix<double,5,5>::Identity();


    // 注意！！！*******************************************//////
    INp=MatrixXd::Identity(prediction_horizon_Np,prediction_horizon_Np);
    INc=MatrixXd::Identity(control_horizon_Nc,control_horizon_Nc);
    QQ = kroneckerProduct(INp, Q);
    RR = kroneckerProduct(INc, R);
    A_I.resize(control_horizon_Nc*5,control_horizon_Nc*5);
    V_T.resize(control_horizon_Nc*5,1);

    for(int i=0;i<control_horizon_Nc;i++)
        for(int j=0;j<control_horizon_Nc;j++)
            A_I.block(i*5,j*5,5,5)=(j<=i)?I2:O;

    for(int i=0;i<control_horizon_Nc;i++)
        V_T.block(i*5,0,5,1)=Matrix<double, 5, 1>::Zero();

    V_max0.resize(control_horizon_Nc*5,1);
    V_min0.resize(control_horizon_Nc*5,1);
    U_max0.resize(control_horizon_Nc*5,1);
    U_min0.resize(control_horizon_Nc*5,1);
    for(int i=0;i<control_horizon_Nc;i++){
        V_max0.block(i*5,0,5,1)=V_max;
        V_min0.block(i*5,0,5,1)=V_min;
        U_max0.block(i*5,0,5,1)=U_max;
        U_min0.block(i*5,0,5,1)=U_min;
    }

}

Eigen::Matrix<double, 6, 1> MPC::ComputeDesiredVelocity(
        double t,
        double T,
        double t_max,
        const Eigen::Matrix<double, 6, 1>& current_Eta,
        const Eigen::Matrix<double, 6, 1>& current_V,
        const vector<Eigen::Matrix<double, 5, 1>>& reference_Eta) {

    Eta<<current_Eta(0),current_Eta(1),current_Eta(2),current_Eta(4),current_Eta(5);
    V<<current_V(0),current_V(1),current_V(2),current_V(4),current_V(5);
    Eta_d=reference_Eta[std::floor(t/T)+1];

    V_d=(Eta_d-Eta)/T;
    /*
    for(int i=0;i<5;i++){
        V_d(i)=V_d(i)<V_min(i)?V_min(i):V_d(i);
        V_d(i)=V_d(i)>V_max(i)?V_max(i):V_d(i);
    }
*/
    u_d=V_d(0);
    v_d=V_d(1);
    w_d=V_d(2);
    theta_d=V_d(3);
    psi_d=V_d(4);

    a14=-u_d*sin(theta_d)*cos(psi_d)+w_d*cos(theta_d)*cos(psi_d);
    a24=-u_d*sin(theta_d)*sin(psi_d)+w_d*cos(theta_d)*sin(psi_d);
    a34=-u_d*cos(theta_d)-w_d*sin(theta_d);
    a15=-u_d*cos(theta_d)*sin(psi_d)-v_d*cos(psi_d)-w_d*sin(theta_d)*sin(psi_d);
    a25=u_d*cos(theta_d)*cos(psi_d)-v_d*sin(psi_d)+w_d*sin(theta_d)*cos(psi_d);
    b11=cos(theta_d)*cos(psi_d);
    b12=-sin(psi_d);
    b13=sin(theta_d)*cos(psi_d);
    b21=cos(theta_d)*sin(psi_d);
    b22=cos(psi_d);
    b23=sin(theta_d)*sin(psi_d);
    b31=-sin(theta_d);
    b33=cos(theta_d);
    b55=1/cos(theta_d);

    a<< 0,0,0,a14,a15,
        0,0,0,a24,a25,
        0,0,0,a34,0,
        0,0,0,0,0,
        0,0,0,0,0;
    b<< b11,b12,b13,0,0,
        b21,b22,b23,0,0,
        b31,0,b33,0,0,
        0,0,0,1,0,
        0,0,0,0,b55;
    al=a*T+I;
    bl=b*T;

    A<<al,bl,O,I;
    B<<bl,I;
    C<<I,O;

    if((t_max-t)/T>=prediction_horizon_Np || (t_max-t)/T>=control_horizon_Nc){
        prediction_horizon_Np=(t_max-t)/T>=prediction_horizon_Np?prediction_horizon_Np:(t_max-t)/T;
        control_horizon_Nc=(t_max-t)/T>=control_horizon_Nc?control_horizon_Nc:(t_max-t)/T;
        update_outloop_params(prediction_horizon_Np,control_horizon_Nc);
    }

    MatrixXd PSI(prediction_horizon_Np*5, 10);
    MatrixXd THETA(prediction_horizon_Np*5,control_horizon_Nc*5);
    MatrixXd U(control_horizon_Nc*5,1);
    MatrixXd x_k(10,1);
    for(int i=0;i<prediction_horizon_Np;i++)
        PSI.block(i*5,0,5,10)=C*Msqrt(A,i+1);
    for(int i=0;i<prediction_horizon_Np;i++)
        for(int j=0;j<control_horizon_Nc;j++)
            THETA.block(i*5,j*5,5,5)=C*Msqrt(A,i-j)*B;

    for(int i=0;i<control_horizon_Nc;i++)
        U.block(i*5,0,5,1)=V-V_d;

    x_k.block(0,0,5,1)=(Eta-Eta_d);
    x_k.block(5,0,5,1)=V-V_d;

    E=PSI*x_k;
    H=THETA.transpose()*QQ*THETA+RR;
    g=E.transpose()*QQ*THETA;

    // 求解 QP 问题
    VectorXd U_star(5*control_horizon_Nc);
    solveQP(H, g,A_I,V_max0-V_T,-V_min0+V_T, U_max0,U_min0, U_star);
    U_star0<<U_star(0),U_star(1),U_star(2), 0 , U_star(3),U_star(4);

    return U_star0+current_V;
}
void MPC::solveQP(const MatrixXd& H, const MatrixXd& g, const MatrixXd& A_I0, const MatrixXd& V_max, const MatrixXd& V_min,
                const MatrixXd& U_max, const MatrixXd& U_min,  VectorXd& U_star){
/*
    int AIrows=A_I0.rows();
    int AIcols=A_I0.cols();
    MatrixXd A_I(AIrows*2,AIcols);
    A_I<<A_I0,-A_I0;*/
    // 定义 QP 问题
    int nVar = H.rows();  // 变量数量
    int nC = A_I0.rows();  // 约束数量

    // 创建 QP 问题实例
    qpOASES::QProblem qp(nVar, nC);

    // 设置 QP 选项

    qpOASES::Options options;
    options.setToMPC();
    options.enableEqualities = qpOASES::BT_TRUE;  // 允许等式约束
    options.terminationTolerance = 0.1;  // 调整终止容差

    //options.maxIter = 1000;  // 增加最大迭代次数
    qp.setOptions(options);

    // 定义 QP 问题的参数
    // 定义 QP 问题的参数
    Eigen::MatrixXd H_copy = H;
    qpOASES::real_t* H_qp = H_copy.data();
    Eigen::MatrixXd g_copy = g;
    qpOASES::real_t* g_qp = g_copy.data();
    Eigen::MatrixXd A_copy = A_I0;
    qpOASES::real_t* A_qp = A_copy.data();
    Eigen::MatrixXd lb_copy = U_min;
    qpOASES::real_t* lb = lb_copy.data();
    Eigen::MatrixXd ub_copy = U_max;
    qpOASES::real_t* ub = ub_copy.data();

    //qpOASES::real_t* lbA = (V_min - V_T).data();
    //qpOASES::real_t* ubA = (V_max - V_T).data();
    Eigen::MatrixXd lbA_copy = -V_min;
    qpOASES::real_t* lbA = lbA_copy.data();
    Eigen::MatrixXd ubA_copy = V_max;
    qpOASES::real_t* ubA = ubA_copy.data();

    // 初始化 QP 问题
    int nWSR = 1000;  // 最大迭代次数
    qpOASES::returnValue status = qp.init(H_qp, g_qp, A_qp, lb, ub, lbA, ubA, nWSR);


    // 检查 QP 求解状态
    if (status != qpOASES::SUCCESSFUL_RETURN) {
        std::cerr << "QP 求解失败: " << status << std::endl;
        return;
    }
    // 获取最优解
    qp.getPrimalSolution(U_star.data());


}
MatrixXd MPC::Msqrt(MatrixXd A,int n){
    // 获取矩阵的行数
    int rows = A.rows();
    // 获取矩阵的列数
    int cols = A.cols();

    if(rows!=cols){
        std::cerr << "矩阵 A 不是方阵，无法创建同维度的单位矩阵。" << std::endl;
        return Eigen::MatrixXd::Identity(rows, rows);  // 程序异常终止
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(rows, rows);

    if(n==0)
        return I;
    if(n<0)
        return I*0;
    for(int i=1;i<n;i++)
        I*=A;
    return I;
}
Eigen::MatrixXd MPC::kroneckerProduct(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    int rowsA = A.rows();
    int colsA = A.cols();
    int rowsB = B.rows();
    int colsB = B.cols();

    // 创建正确的矩阵
    Eigen::MatrixXd C(rowsA * rowsB, colsA * colsB);

    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsA; ++j) {
            for (int k = 0; k < rowsB; ++k) {
                for (int l = 0; l < colsB; ++l) {
                    // 使用正确的元素访问方式
                    C(i * rowsB + k, j * colsB + l) = A(i, j) * B(k, l);
                }
            }
        }
    }

    return C;
}
void MPC::update_outloop_params(int prediction_horizon_Np,int control_horizon_Nc){
    INp=MatrixXd::Identity(prediction_horizon_Np,prediction_horizon_Np);
    INc=MatrixXd::Identity(control_horizon_Nc,control_horizon_Nc);
    QQ = kroneckerProduct(INp, Q);
    RR = kroneckerProduct(INc, R);
    A_I.resize(control_horizon_Nc*5,control_horizon_Nc*5);
    V_T.resize(control_horizon_Nc*5,1);

    for(int i=0;i<control_horizon_Nc;i++)
        for(int j=0;j<control_horizon_Nc;j++)
            A_I.block(i*5,j*5,5,5)=(j<=i)?I2:O;

    for(int i=0;i<control_horizon_Nc;i++)
        V_T.block(i*5,0,5,1)=Matrix<double, 5, 1>::Zero();

    V_max0.resize(control_horizon_Nc*5,1);
    V_min0.resize(control_horizon_Nc*5,1);
    U_max0.resize(control_horizon_Nc*5,1);
    U_min0.resize(control_horizon_Nc*5,1);
    for(int i=0;i<control_horizon_Nc;i++){
        V_max0.block(i*5,0,5,1)=V_max;
        V_min0.block(i*5,0,5,1)=V_min;
        U_max0.block(i*5,0,5,1)=U_max;
        U_min0.block(i*5,0,5,1)=U_min;
    }
}

