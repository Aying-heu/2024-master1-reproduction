#pragma once
#include"../A_include.h"

struct Time {double ttime=0;};
struct Location_xyz {Vector3d trans = Vector3d(0, 0, 0);};
struct Location_pqr {Vector3d trans = Vector3d(0, 0, 0);};
struct Speed_xyz {Vector3d trans = Vector3d(0, 0, 0);};
struct Angular_speed_pqr {Vector3d trans = Vector3d(0, 0, 0);};

ostream &operator<<(ostream &out, const Time &t);
ostream &operator<<(ostream &out, const Location_xyz &t);
ostream &operator<<(ostream &out, const Location_pqr &t) ;
ostream &operator<<(ostream &out, const Speed_xyz &t) ;
ostream &operator<<(ostream &out, const Angular_speed_pqr &t);

istream &operator>>(istream &in, Time &t) ;
istream &operator>>(istream &in, Location_xyz &t) ;
istream &operator>>(istream &in, Location_pqr &t) ;
istream &operator>>(istream &in, Speed_xyz &t) ;
istream &operator>>(istream &in, Angular_speed_pqr &t);

class VISUALIZE{
public:


        pangolin::OpenGlRenderState s_cam_follow;
        pangolin::View d_cam_follow;

        pangolin::OpenGlRenderState s_cam_overall;
        pangolin::View d_cam_overall;


        VISUALIZE();
        void Init();
        void update_information(double& t,double& x,double& y,double& z,double& phi,double& theta,double& psi,double& u,double& v,double& w,double& p,double& q,double& r);
        void Draw(int mode,
                double& t,double& T,
                vector<Matrix<double, 3, 1>>& position_list,
                vector<Matrix<double, 3, 1>>& pose_list,
                vector<Matrix<double, 3, 1>>& speed_list,
                vector<Matrix<double, 3, 1>>& angular_speed_list,
                vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses);
        void Draw_current(double& t,double& T,
                vector<Matrix<double, 3, 1>>& position_list,
                vector<Matrix<double, 3, 1>>& pose_list,
                vector<Matrix<double, 3, 1>>& speed_list,
                vector<Matrix<double, 3, 1>>& angular_speed_list,
                vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses);
        void Draw_final(double& t,double& T,
                vector<Matrix<double, 3, 1>>& position_list,
                vector<Matrix<double, 3, 1>>& pose_list,
                vector<Matrix<double, 3, 1>>& speed_list,
                vector<Matrix<double, 3, 1>>& angular_speed_list,
                vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses);
        void Draw_coord();
        void Draw_Trace(vector<Matrix<double, 5, 1>>&Trace);

        void drawScene(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses,
               pangolin::OpenGlRenderState &s_cam,pangolin::View &d_cam);
        void drawScene_final(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses,
                pangolin::OpenGlRenderState &s_cam_follow,pangolin::View &d_cam_follow,
                pangolin::OpenGlRenderState &s_cam_overall,pangolin::View &d_cam_overall);
private:
        int Width;
        int Height;
        double x_min,x_max,y_min,y_max,z_min,z_max;
        pangolin::Var<Time> time;
        pangolin::Var<Location_xyz> location_xyz;
        pangolin::Var<Location_pqr> location_pqr;
        pangolin::Var<Speed_xyz> speed_xyz;
        pangolin::Var<Angular_speed_pqr> angular_speed_pqr;

};

