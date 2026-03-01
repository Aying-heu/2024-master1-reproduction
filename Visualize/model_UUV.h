#pragma once
#include"../A_include.h"

struct Vector_propeller{
    Vector3d mid;
    Matrix3d ang;
    vector<Vector3d> up;
    vector<Vector3d> down;
};
struct Body{
    vector<Matrix<double,3,1>> point;
};

class UUV_MODEL {
public:
UUV_MODEL();
    double c,b,d,f,m,n,alpha;
    Matrix<double,6,8>B;
    Vector3d pos1;
    Vector3d pos2;
    Vector3d pos3;
    Vector3d pos4;
    Vector3d pos5;
    Vector3d pos6;
    Vector3d pos7;
    Vector3d pos8;
    double radius=0.02*2;
    double height=0.01*2;

    AngleAxisd rotation_vector;
    AngleAxisd rotation_vector1;
    AngleAxisd rotation_vector2;
    AngleAxisd rotation_vector3;
    AngleAxisd rotation_vector4;
    AngleAxisd rotation_vector_N;

    Matrix3d ang1;
    Matrix3d ang2;
    Matrix3d ang3;
    Matrix3d ang4;
    Matrix3d ang5;
    Matrix3d ang6;
    Matrix3d ang7;
    Matrix3d ang8;

    Vector_propeller V1;
    Vector_propeller V2;
    Vector_propeller V3;
    Vector_propeller V4;
    Vector_propeller V5;
    Vector_propeller V6;
    Vector_propeller V7;
    Vector_propeller V8;

    AngleAxisd rotation_vector_Slow;
    Matrix3d ang_Slow0;
    Matrix3d ang_Slow;

    Vector_propeller generate_V(Vector3d pos,Matrix3d ang);
    Vector_propeller generate_V(Vector3d pos,Matrix3d ang,double radius,double height);

    void generate_B(Body& B);

    void Draw_V(Vector_propeller& V,Isometry3d& pose);
    void Draw_B(Isometry3d& pose);
    void Draw_UUV(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses);
};







