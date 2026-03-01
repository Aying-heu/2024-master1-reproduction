#include"visualize.h"

ostream &operator<<(ostream &out, const Time &t) {
        out << std::fixed << std::setprecision(4); // 设置小数位数为2
        out << "Time= " << t.ttime<< " s";
        out << std::defaultfloat; // 恢复默认的小数输出格式
        return out;}
ostream &operator<<(ostream &out, const Location_xyz &t) {
        out << std::fixed << std::setprecision(4); // 设置小数位数为4
        out << "=[" << t.trans(0) << ",  " << t.trans(1) << ",  " << t.trans(2) << "] m";
        out << std::defaultfloat; // 恢复默认的小数输出格式
        return out;}
ostream &operator<<(ostream &out, const Location_pqr &t) {
        out << std::fixed << std::setprecision(2); // 设置小数位数为2
        out << "=[" << t.trans(0) << "," << t.trans(1) << "," << t.trans(2) << "]rad = ["<< t.trans(0)/PI*180 << "°," << t.trans(1)/ PI * 180 << "°," << t.trans(2) / PI * 180 << "°]";
        out << std::defaultfloat; // 恢复默认的小数输出格式
        return out;}
ostream &operator<<(ostream &out, const Speed_xyz &t) {
        out << std::fixed << std::setprecision(4); // 设置小数位数为4
        out << "=[" << t.trans(0) << ",  " << t.trans(1) << ",  " << t.trans(2) << "] m/s";
        out << std::defaultfloat; // 恢复默认的小数输出格式
        return out;}
ostream &operator<<(ostream &out, const Angular_speed_pqr &t) {
        out << std::fixed << std::setprecision(2); // 设置小数位数为4
        out << "=[" << t.trans(0) << "," << t.trans(1) << "," << t.trans(2) << "]rad/s = ["<< t.trans(0)/PI*180 << "," << t.trans(1)/PI*180 << "," << t.trans(2)/PI*180 <<"]度/s";
        out << std::defaultfloat; // 恢复默认的小数输出格式
        return out;}

istream &operator>>(istream &in, Time &t) {return in;}
istream &operator>>(istream &in, Location_xyz &t) {return in;}
istream &operator>>(istream &in, Location_pqr &t) {return in;}
istream &operator>>(istream &in, Speed_xyz &t) {return in;}
istream &operator>>(istream &in, Angular_speed_pqr &t) {return in;}

VISUALIZE::VISUALIZE() :
    Width(1900),Height(560),
    x_min(0),x_max(0),y_min(0),y_max(0),z_min(0),z_max(0),

    time("ui.time", Time()),
    location_xyz("ui.location", Location_xyz()),
    location_pqr("ui.angular", Location_pqr()),
    speed_xyz("ui.speed", Speed_xyz()),
    angular_speed_pqr("ui.angular_speed", Angular_speed_pqr()){

}

void VISUALIZE::Draw(int mode,
    double& t,double& T,
    vector<Matrix<double, 3, 1>>& position_list,
    vector<Matrix<double, 3, 1>>& pose_list,
    vector<Matrix<double, 3, 1>>& speed_list,
    vector<Matrix<double, 3, 1>>& angular_speed_list,
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses)
{
    if(mode==0)
        Draw_final(t, T,position_list,pose_list,speed_list,angular_speed_list,poses);
    else if(mode==1)
        Draw_current(t, T,position_list,pose_list,speed_list,angular_speed_list,poses);
}

void VISUALIZE::Init(){
/*
// create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1000, 800);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam=pangolin::OpenGlRenderState(
    pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0.8, 0.4, -0.4, 0, 0, 0, 0, 0, -1)
    );

    d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1000.0f / 800.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

    // SetBounds(1,2,3,4);左下角开始。1.往上多少开始 2.往上到哪 3.往右多少开始 4.往右到多少
    pangolin::CreatePanel("ui").SetBounds(0.55, 1.0, 0.0,0.5);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 10000),
            pangolin::ModelViewLookAt(

                (-x_min + x_max)*1.05+0.8 , (-y_min + y_max)*1.05+0.4, (z_max-z_min)*1.05-0.4,
                                    (x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2,

               position_list.back()(0)+0.8, position_list.back()(1)+0.4, position_list.back()(2)-0.4,
                                      position_list.back()(0), position_list.back()(1), position_list.back()(2),
                                      0, 0, -1)
        );
*/

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Gooooooooooooood Ohhhhhhhhhhhhhhhhhhh", Width, Height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam_follow=pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0.601,-0.402, -0.403,
                                +0.003, -0.002, -0.001,
                                    0,0,-1)
        );



    d_cam_follow = pangolin::CreateDisplay()
      .SetBounds(0, 1.0, 0.26,0.63, -500.0f / 400.0f)   //右上角  500*400
      .SetHandler(new pangolin::Handler3D(s_cam_follow));

    s_cam_overall=pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0.601, -0.402, -0.403,
                                    0.003, -0.002, -0.001,
                                    0,0,-1)
        );

    d_cam_overall = pangolin::CreateDisplay()
      .SetBounds(0.0, 1, 0.63, 1.0, -500.0f / 400.0f)   //右下角  500*400
      .SetHandler(new pangolin::Handler3D(s_cam_overall));

    // SetBounds(1,2,3,4);左下角开始。1.往上多少开始 2.往上到哪 3.往右多少开始 4.往右到多少
    pangolin::CreatePanel("ui").SetBounds(0, 1.0, 0.0,0.26);

}
void VISUALIZE::update_information(double& t,double& x,double& y,double& z,double& phi,double& theta,double& psi,double& u,double& v,double& w,double& p,double& q,double& r){

    Time time0;
    time0.ttime=t;
    time=time0;

    Location_xyz XYZ;
    XYZ.trans=Matrix<double,3,1>(x,y,z);
    location_xyz=XYZ;

    Location_pqr PQR;
    PQR.trans=Vector3d(phi,theta,psi);
    location_pqr=PQR;

    Speed_xyz S_XYZ;
    S_XYZ.trans=Vector3d(u,v,w);
    speed_xyz=S_XYZ;

    Angular_speed_pqr AS_PQR;
    AS_PQR.trans=Vector3d(p,q,r);
    angular_speed_pqr=AS_PQR;


}
UUV_MODEL uuv_model;
PARAMS params;
void VISUALIZE::Draw_current(double& t,double& T,
        vector<Matrix<double, 3, 1>>& position_list,
        vector<Matrix<double, 3, 1>>& pose_list,
        vector<Matrix<double, 3, 1>>& speed_list,
        vector<Matrix<double, 3, 1>>& angular_speed_list,
        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses)
{

    x_min=x_min<position_list.back()(0)?x_min:position_list.back()(0);
    y_min=y_min<position_list.back()(1)?y_min:position_list.back()(1);
    z_min=z_min<position_list.back()(2)?z_min:position_list.back()(2);

    x_max=x_max>position_list.back()(0)?x_max:position_list.back()(0);
    y_max=y_max>position_list.back()(1)?y_max:position_list.back()(1);
    z_max=z_max>position_list.back()(2)?z_max:position_list.back()(2);


        s_cam_follow = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(poses.back().translation()[0]+0.601, poses.back().translation()[1]-0.402, poses.back().translation()[2]-0.403,
                                    poses.back().translation()[0]+0.003, poses.back().translation()[1]-0.002, poses.back().translation()[2]-0.001,
                                    0,0,-1)
        );

        s_cam_overall = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1000, 800, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt((-x_min + x_max)+0.601, (-y_min + y_max)-0.402, (z_max-z_min)-0.403,
                                    (x_min + x_max) / 2+0.003, (y_min + y_max) / 2-0.002, (z_min + z_max) / 2-0.001,
                                    0,0,-1)
        );

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        drawScene(poses,s_cam_follow,d_cam_follow);
        drawScene(poses,s_cam_overall,d_cam_overall);

        pangolin::FinishFrame();
        //usleep(500);   // sleep 0.5 ms
}
void VISUALIZE::Draw_final(double& t,double& T,
        vector<Matrix<double, 3, 1>>& position_list,
        vector<Matrix<double, 3, 1>>& pose_list,
        vector<Matrix<double, 3, 1>>& speed_list,
        vector<Matrix<double, 3, 1>>& angular_speed_list,
        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses)
{
    drawScene_final(poses,
                    s_cam_follow,d_cam_follow,
                    s_cam_overall,d_cam_overall);
}
void VISUALIZE::Draw_coord(){
    glBegin(GL_LINES);
    glColor3f(1,0,0);   glVertex3d(0,0,0);    glVertex3d(0.5,0,0);
    glColor3f(0,1,0);   glVertex3d(0,0,0);    glVertex3d(0,0.5,0);
    glColor3f(0,0,1);   glVertex3d(0,0,0);    glVertex3d(0,0,0.5);
    glEnd();
}
void VISUALIZE::Draw_Trace(vector<Matrix<double, 5, 1>>&Trace){
    glColor4f(0.5, 0, 0.5,0.3);
    glBegin(GL_LINES);
    int di=20;
    for (int i = 0;i <= Trace.size()-2-di;i+=di) {
        glVertex3d(Trace[i](0),Trace[i](1),Trace[i](2));
        glVertex3d(Trace[i+di](0),Trace[i+di](1),Trace[i+di](2));
    }
    glEnd();
}


void VISUALIZE::drawScene(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses,
               pangolin::OpenGlRenderState &s_cam,pangolin::View &d_cam){
                   d_cam.Activate(s_cam);
    //drawScene(poses);
    glLineWidth(2);
    int Draw_num=poses.size();
    for (size_t i = (Draw_num-100>0?Draw_num-100:0); i < Draw_num; i++) {
        // 画每个位姿的三个坐标轴
        Vector3d Ow = poses[i].translation();
        Vector3d Xw = poses[i] * (0.05 * Vector3d(1, 0, 0));
        Vector3d Yw = poses[i] * (0.05 * Vector3d(0, 1, 0));
        Vector3d Zw = poses[i] * (0.05 * Vector3d(0, 0, 1));
        glBegin(GL_LINES);
        glColor4f(1.0, 0.0, 0.0,0.3);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Xw[0], Xw[1], Xw[2]);
        glColor4f(0.0, 1.0, 0.0,0.3);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Yw[0], Yw[1], Yw[2]);
        glColor4f(0.0, 0.0, 1.0,0.3);
        glVertex3d(Ow[0], Ow[1], Ow[2]);
        glVertex3d(Zw[0], Zw[1], Zw[2]);
        glEnd();
    }
    // 画出连线
    int Final_Draw_num=1000;

    auto p1 = poses[0];
    glColor3f(0.3, 0.3, 0.3);
    glBegin(GL_LINES);
    for (size_t i = 0;i <= Draw_num;) {

        auto p2 = poses[i];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        p1=p2;

        i+=(i <= Draw_num-Final_Draw_num)?int((Draw_num-Final_Draw_num)/5000)+1:1;
    }
    glEnd();
    glColor3f(0.8, 0.8, 0.8);
    glBegin(GL_LINES);
    glVertex3d(poses[0].translation()[0], poses[0].translation()[1], poses[0].translation()[2]);
    glVertex3d(poses.back().translation()[0], poses.back().translation()[1], poses.back().translation()[2]);
    glEnd();


    uuv_model.Draw_UUV(poses);
    Draw_coord();
    Draw_Trace(params.Reference_Eta);
}
void VISUALIZE::drawScene_final(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses,
                pangolin::OpenGlRenderState &s_cam_follow,pangolin::View &d_cam_follow,
                pangolin::OpenGlRenderState &s_cam_overall,pangolin::View &d_cam_overall){
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        drawScene(poses,s_cam_follow,d_cam_follow);
        drawScene(poses,s_cam_overall,d_cam_overall);

        uuv_model.Draw_UUV(poses);
        Draw_coord();
        Draw_Trace(params.Reference_Eta);
        pangolin::FinishFrame();
        usleep(100);   // sleep    5000 = 5 ms    dt秒=dt*1000 ms  = dt*1000*1000
    }
}
