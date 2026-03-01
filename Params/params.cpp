#include"params.h"

PARAMS::PARAMS() :
    m(177),
    u(0), v(0), w(0), p(0), q(0), r(0),
    x(0), y(0), z(0), phi(0), theta(0), psi(0),
    X(0), Y(0), Z(0), K(0), M(0), N(0),
    W(m*9.8), B(1158),
    xg(0),yg(0),xb(0),yb(0),
    zg(0.1),zb(-0.05),
    t(0), T(0.005), t_max(100){

    target<<0.5,0.5,-0.5,PI/4,PI/4,PI/4;

    V<<u,v,w,p,q,r;
    V_dot<<0,0,0,0,0,0;
    Eta<<x,y,z,psi,theta,psi;
    Eta_dot<<0,0,0,0,0,0;
    Tau<<X,Y,Z,K,M,N;

    for(double tt=0;tt<=t_max;tt+=T){
        Matrix<double, 5, 1>e;
        //e<<2*sin(0.8*tt),2-2*cos(0.8*tt),0.2*tt,0.1*sin(0.8*tt),0.1-0.1*cos(0.8*tt);

        e<<     0.5*sin(0.8*tt),
                0.5-0.5*cos(0.8*tt),
                0.3*tt,0,0;

        Times.push_back(tt);
        Reference_Eta.push_back(e);
    }


    for(double i=0;i<=t_max;i+=T){
        //tau_E.push_back(0);
        tau_E.push_back(0.5*sin(i*75)*cos(i/150));
    }


    Rx <<   1,0,0,
            0,cos(phi),sin(phi),
            0,-sin(phi),cos(phi) ;
    Ry <<   cos(theta),0,-sin(theta),
            0,1,0,
            sin(theta),0,cos(theta) ;
    Rz <<   cos(psi),sin(psi),0,
            -sin(psi),cos(psi),0,
            0,0,1 ;
    J1_eta2 = Rx*Ry*Rz;
    J2_eta2<<   1,sin(phi) * tan(theta),cos(phi) * tan(theta),
                0,cos(phi),-sin(phi),
                0,sin(phi) / cos(theta),cos(phi) / cos(theta) ;
    J_eta<< J1_eta2,zeroMatrix,zeroMatrix,J2_eta2;


    zeroMatrix<<0,0,0,0,0,0,0,0,0;
    // 初始化其他矩阵成员等，以下是示例，需根据实际逻辑调整
    M_RB << 117, 0, 0, 0, 0, 0,
            0, 117, 0, 0, 0, 0,
            0, 0, 117, 0, 0, 0,
            0, 0, 0, 10.7, 0, 0,
            0, 0, 0, 0, 11.8, 0,
            0, 0, 0, 0, 0, 13.4;

    M_A << 58.4, 0, 0, 0, 0, 0,
            0, 23.8, 0, 0, 0, 0,
            0, 0, 23.8, 0, 0, 0,
            0, 0, 0, 3.38, 0, 0,
            0, 0, 0, 0, 1.18, 0,
            0, 0, 0, 0, 0, 2.67;

    M_ = M_RB + M_A;

    double a1=58.4*u;
    double a2=23.8*v;
    double a3=23.8*w;
    double b1=3.38*p;
    double b2=1.18*q;
    double b3=2.67*r;
    CA_V<<  0,0,0,0,-a3,a2,
            0,0,0,a3,0,-a1,
            0,0,0,-a2,a1,0,
            0,-a3,a2,0,-b3,b2,
            a3,0,-a1,b3,0,-b1,
            -a2,a1,0,-b2,b1,0;
    CRB_V<< 0,0,0,0,m*w,m*v,
            0,0,0,-m*w,0,m*u,
            0,0,0,m*v,-m*u,0,
            0,m*w,-m*v,0,13.4*r,11.8*q,
            -m*w,0,m*u,-13.4*r,0,10.7*p,
            -m*v,-m*u,0,-11.8*q,-10.7*p,0;
    C_V=CA_V+CRB_V;
    // 继续初始化其他矩阵成员，以下代码中的赋值需根据实际逻辑核对
    D_V << 120 + 90 * fabs(u), 0, 0, 0, 0, 0,
            0, 90 + 90 * fabs(v), 0, 0, 0, 0,
            0, 0, 150 + 120 * fabs(w), 0, 0, 0,
            0, 0, 0, 50 + 10 * fabs(p), 0, 0,
            0, 0, 0, 0, 15 + 12 * fabs(q), 0,
            0, 0, 0, 0, 0, 18 + 15 * fabs(r);

    g_Eta<< (W-B)*sin(theta),
            -(W-B)*sin(phi)*cos(theta),
            -(W-B)*cos(phi)*cos(theta),
            -(yg*W-yb*B)*cos(phi)*cos(theta)+(zg*W-xb*B)*sin(phi)*cos(theta),
            (zg*W-zb*B)*sin(theta)+(xg*W-xb*B)*cos(phi)*cos(theta),
            -(xg*W-xb*B)*cos(phi)*cos(theta)-(yg*W-yb*B)*sin(theta);


    pose = Isometry3d::Identity();
}

void PARAMS::updateV_u() {u=V(0);v=V(1);w=V(2);p=V(3);q=V(4);r=V(5);}
void PARAMS::updateEta_x() {x=Eta(0);y=Eta(1);z=Eta(2);phi=Eta(3);theta=Eta(4);psi=Eta(5);}
void PARAMS::update_Tau_X() {X=Tau(0);Y=Tau(1);Z=Tau(2);K=Tau(3);M=Tau(4);N=Tau(5);}

void PARAMS::update_J(){
    Rx <<1,0,0,0,cos(phi),sin(phi),0,-sin(phi),cos(phi) ;
    Ry <<cos(theta),0,-sin(theta),0,1,0,sin(theta),0,cos(theta) ;
    Rz <<cos(psi),sin(psi),0,-sin(psi),cos(psi),0,0,0,1 ;
    J1_eta2 = Rx*Ry*Rz;
    J2_eta2<< 1,sin(phi) * tan(theta),cos(phi) * tan(theta),0,cos(phi),-sin(phi),0,sin(phi) / cos(theta),cos(phi) / cos(theta) ;
    J_eta<< J1_eta2,zeroMatrix,zeroMatrix,J2_eta2;
}
void PARAMS::update_CV(){
    double a1=58.4*u;double a2=23.8*v;double a3=23.8*w;
    double b1=3.38*p;double b2=1.18*q;double b3=2.67*r;
    CA_V<<0,0,0,0,-a3,a2,0,0,0,a3,0,-a1,0,0,0,-a2,a1,0,0,-a3,a2,0,-b3,b2,a3,0,-a1,b3,0,-b1,-a2,a1,0,-b2,b1,0;
    CRB_V<<0,0,0,0,m*w,m*v,0,0,0,-m*w,0,m*u,0,0,0,m*v,-m*u,0,0,m*w,-m*v,0,13.4*r,11.8*q,-m*w,0,m*u,-13.4*r,0,10.7*p,-m*v,-m*u,0,-11.8*q,-10.7*p,0;
    C_V=CA_V+CRB_V;
}
void PARAMS::update_DV(){
    D_V << 120 + 90 * fabs(u), 0, 0, 0, 0, 0,0, 90 + 90 * fabs(v), 0, 0, 0, 0,0, 0, 150 + 120 * fabs(w), 0, 0, 0,0, 0, 0, 50 + 10 * fabs(p), 0, 0,0, 0, 0, 0, 15 + 12 * fabs(q), 0,0, 0, 0, 0, 0, 18 + 15 * fabs(r);

}
void PARAMS::update_g_Eta(){g_Eta<< (W-B)*sin(theta),-(W-B)*sin(phi)*cos(theta),-(W-B)*cos(phi)*cos(theta),-(yg*W-yb*B)*cos(phi)*cos(theta)+(zg*W-xb*B)*sin(phi)*cos(theta),(zg*W-zb*B)*sin(theta)+(xg*W-xb*B)*cos(phi)*cos(theta),-(xg*W-xb*B)*cos(phi)*cos(theta)-(yg*W-yb*B)*sin(theta);}


void PARAMS::update_position_list(){
    list_init<<Eta(0),Eta(1),Eta(2);
    position_list.push_back(list_init);
}
void PARAMS::update_pose_list(){
    list_init<<Eta(3),Eta(4),Eta(5);
    pose_list.push_back(list_init);
}
void PARAMS::update_speed_list(){
    list_init<<V(0),V(1),V(2);
    speed_list.push_back(list_init);
}
void PARAMS::update_angular_speed_list(){
    list_init<<V(3),V(4),V(5);
    angular_speed_list.push_back(list_init);
}
void PARAMS::update_pose(){
    // 按照Z - Y - X顺序创建旋转，即先绕x轴旋转roll，再绕y轴旋转pitch，最后绕z轴旋转yaw
    AngleAxisd rollAngle(phi, Vector3d::UnitX());
    AngleAxisd pitchAngle(theta, Vector3d::UnitY());
    AngleAxisd yawAngle(psi, Vector3d::UnitZ());
    // 组合旋转
    Quaterniond q = yawAngle * pitchAngle * rollAngle;
    // 组合旋转得到3x3旋转矩阵
    Matrix3d rotationMatrix = q.toRotationMatrix();
    // 创建4x4欧式变换矩阵，假设平移为零向量
    Isometry3d pose = Isometry3d::Identity();
    pose.rotate(rotationMatrix);
    Vector3d location(x,y,z);
    pose.pretranslate(location);
    poses.push_back(pose);

    Eta_list.push_back(Eta);
    Tau_list.push_back(Tau);
}

VISUALIZE Visual;
void PARAMS::Update_Params(){
    updateV_u();
    updateEta_x();
    update_Tau_X();
    update_J();
    update_CV();
    update_DV();
    update_g_Eta();
    update_position_list();
    update_pose_list();
    update_speed_list();
    update_angular_speed_list();

    Visual.update_information( t, x, y, z, phi, theta, psi, u, v, w, p, q, r);
    update_pose();
}
void PARAMS::Write_data_v(vector<double>& data,string filename){
    // 创建文件输出流
    std::ofstream outfile(filename);
    // 检查文件是否成功打开
    if (!outfile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
    }
    // 将vector中的数据逐行写入文件
    for (const auto& element : data) {
        outfile << element << std::endl;
    }
    // 关闭文件输出流
    outfile.close();
    std::cout << "数据已成功写入文件 " << filename << std::endl;
}
void PARAMS::Write_data_vv(vector<Matrix<double,5,1>>& data, const std::string& filename){
    std::ofstream outfile(filename);
    if (!outfile.is_open())
        std::cerr << "无法打开文件: " << filename << std::endl;
    // 遍历外层 vector
    for (const auto& sub_vector : data) {
        // 遍历内层 vector
        for (size_t i = 0; i < sub_vector.size(); ++i) {
            outfile << sub_vector(i);
            if (i < sub_vector.size() - 1) {
                outfile << " ";  // 元素间用空格分隔
            }
        }
        outfile << std::endl;  // 不同行的数据使用换行符分隔
    }
    outfile.close();
    std::cout << "数据已成功写入文件 " << filename << std::endl;
}
void PARAMS::Write_data_vv(vector<Matrix<double,6,1>>& data, const std::string& filename){
    std::ofstream outfile(filename);
    if (!outfile.is_open())
        std::cerr << "无法打开文件: " << filename << std::endl;
    // 遍历外层 vector
    for (const auto& sub_vector : data) {
        // 遍历内层 vector
        for (size_t i = 0; i < sub_vector.size(); ++i) {
            outfile << sub_vector(i);
            if (i < sub_vector.size() - 1) {
                outfile << " ";  // 元素间用空格分隔
            }
        }
        outfile << std::endl;  // 不同行的数据使用换行符分隔
    }
    outfile.close();
    std::cout << "数据已成功写入文件 " << filename << std::endl;
}

MPC mpc;
void PARAMS::Calculate(int mode){
    vector<string>filenames={"../B_Times.txt","../B_Eta_Tau_E.txt","../B_Reference_Eta_Tau_E.txt","../B_Tau_Tau_E.txt"};
    if(mode==0){
        for(t=0;t<=t_max;t+=T){
            Update_Params();
            Tau=Controller_trace_to_Force(t,T,t_max,Eta,V,Reference_Eta);
            V_dot=dynamics(M_,C_V,D_V,g_Eta,Tau,tau_E[int(t/T)],V);
            V+=V_dot*T;
            Eta_dot=kinematics(J_eta,V);
            Eta+=Eta_dot*T;
        }/*
        Write_data_v(Times,filenames[0]);
        Write_data_vv(Eta_list,filenames[1]);
        Write_data_vv(Reference_Eta,filenames[2]);
        Write_data_vv(Tau_list,filenames[3]);*/
        Visual.Init();
        while(true)
            Visual.Draw(mode,t,T,position_list,pose_list,speed_list,angular_speed_list,poses);
    }
    if(mode==1){
        Visual.Init();
        for(t=0;t<=t_max;t+=T){
            Update_Params();
            Tau=Controller_trace_to_Force(t,T,t_max,Eta,V,Reference_Eta);
            V_dot=dynamics(M_,C_V,D_V,g_Eta,Tau,tau_E[int(t/T)],V);
            V+=V_dot*T;
            Eta_dot=kinematics(J_eta,V);
            Eta+=Eta_dot*T;
            Visual.Draw(mode,t,T,position_list,pose_list,speed_list,angular_speed_list,poses);
        }
        // Write_data_v(Times,filenames[0]);
        // Write_data_vv(Eta_list,filenames[1]);
        // Write_data_vv(Reference_Eta,filenames[2]);
        // Write_data_vv(Tau_list,filenames[3]);
        while(true)
            Visual.Draw(0,t,T,position_list,pose_list,speed_list,angular_speed_list,poses);
    }
}
