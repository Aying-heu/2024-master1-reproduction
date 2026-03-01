#include"model_UUV.h"

UUV_MODEL::UUV_MODEL():
 c(0.3),b(0.2), d(0.1),f(0.1), m(-0.2),n(0.1),alpha(M_PI/4),
 radius(0.02*2),height(0.01*2)
 {
    B<<cos(alpha),cos(alpha),cos(alpha),cos(alpha),0,0,0,0,
        sin(alpha),-sin(alpha),-sin(alpha),sin(alpha),0,0,0,0,
        0,0,0,0,1,1,1,1,
        0,0,0,0,-f,f,-f,f,
        0,0,0,0,-d,-d,d,d,
        c*sin(alpha)+b*cos(alpha),-c*sin(alpha)-b*cos(alpha),c*sin(alpha)+b*cos(alpha),-c*sin(alpha)-b*cos(alpha),0,0,0,0;

    pos1<<c,-b,m;
    pos2<<c,b,m;
    pos3<<-c,-b,m;
    pos4<<-c,b,m;
    pos5<<d,-f,n;
    pos6<<d,f,n;
    pos7<<-d,-f,n;
    pos8<<-d,f,n;

    V1=generate_V(pos1,ang1,radius,height);
    V2=generate_V(pos2,ang2,radius,height);
    V3=generate_V(pos3,ang3,radius,height);
    V4=generate_V(pos4,ang4,radius,height);
    V5=generate_V(pos5,ang5,radius,height);
    V6=generate_V(pos6,ang6,radius,height);
    V7=generate_V(pos7,ang7,radius,height);
    V8=generate_V(pos8,ang8,radius,height);

    AngleAxisd rotation_vector( M_PI / 2,Vector3d(1,0,0));
    AngleAxisd rotation_vector1(-M_PI/4,Vector3d(0,0,1));//沿z轴旋转-45度
    AngleAxisd rotation_vector2(M_PI/4,Vector3d(0,0,1));//沿z轴旋转45度
    AngleAxisd rotation_vector3(-M_PI/4*3,Vector3d(0,0,1));//沿z轴旋转-135度
    AngleAxisd rotation_vector4(M_PI/4*3,Vector3d(0,0,1));//沿z轴旋转135度
    AngleAxisd rotation_vector_N(0,Vector3d(0,0,1));//不旋

    ang1=rotation_vector1.toRotationMatrix()*rotation_vector.toRotationMatrix();
    ang2=rotation_vector2.toRotationMatrix()*rotation_vector.toRotationMatrix();
    ang3=rotation_vector3.toRotationMatrix()*rotation_vector.toRotationMatrix();
    ang4=rotation_vector4.toRotationMatrix()*rotation_vector.toRotationMatrix();
    ang5=rotation_vector_N.toRotationMatrix();
    ang6=rotation_vector_N.toRotationMatrix();
    ang7=rotation_vector_N.toRotationMatrix();
    ang8=rotation_vector_N.toRotationMatrix();

    AngleAxisd rotation_vector_Slow(M_PI/6,Vector3d(0,0,1));
    ang_Slow0=rotation_vector_Slow.toRotationMatrix();
    ang_Slow=ang_Slow0;
}

Vector_propeller UUV_MODEL::generate_V(Vector3d pos,Matrix3d ang,
                            double radius,double height){
    Vector_propeller V;
    V.mid=pos;
    V.ang=ang;
    Vector3d up;
    Vector3d down;
    for(int i=0;i<14;i++){
        double x=radius*cos(M_PI*i/14*2);
        double y=radius*sin(M_PI*i/14*2);
        double z=height/2;
        up<<x,y,z;
        down<<x,y,-z;
        up=ang*up+pos;
        down=ang*down+pos;
        V.up.push_back(up);
        V.down.push_back(down);
    }
    return V;
}
Vector_propeller UUV_MODEL::generate_V(Vector3d pos,Matrix3d ang){
    return generate_V(pos,ang,0.02*2,0.01*2);
}
void UUV_MODEL::generate_B(Body& B){
    double o=2;
}


void UUV_MODEL::Draw_V(Vector_propeller& V,Isometry3d& pose){
    vector<Vector3d> up=V.up;
    vector<Vector3d> down=V.down;
    for (size_t i = 0; i < up.size(); i++) {
        up[i]=pose.rotation()*up[i]+pose.translation();
        down[i]=pose.rotation()*down[i]+pose.translation();
    }
    // 画出连线
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    auto p1 = up[0], p2 = up[up.size()-1];
    glVertex3d(p1[0], p1[1], p1[2]);
    glVertex3d(p2[0], p2[1], p2[2]);
    glEnd();
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    p1 = down[0], p2 = down[up.size()-1];
    glVertex3d(p1[0], p1[1], p1[2]);
    glVertex3d(p2[0], p2[1], p2[2]);
    glEnd();
    for (size_t i = 1; i < up.size(); i++) {
        glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = up[i], p2 = up[i - 1];
        glVertex3d(p1[0], p1[1], p1[2]);
        glVertex3d(p2[0], p2[1], p2[2]);
        glEnd();
    }
    for (size_t i = 1; i < down.size(); i++) {
        glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = down[i], p2 = down[i - 1];
        glVertex3d(p1[0], p1[1], p1[2]);
        glVertex3d(p2[0], p2[1], p2[2]);
        glEnd();
    }
    for (size_t i = 0; i < down.size(); i++) {
        glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = up[i], p2 = down[i];
        glVertex3d(p1[0], p1[1], p1[2]);
        glVertex3d(p2[0], p2[1], p2[2]);
        glEnd();
    }
    for (size_t i = 0; i < up.size()/2; i++) {
        glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = up[i], p2 = up[i+up.size()/2];
        glVertex3d(p1[0], p1[1], p1[2]);
        glVertex3d(p2[0], p2[1], p2[2]);
        glEnd();
    }
    for (size_t i = 0; i < down.size()/2; i++) {
        glColor3f(0.0, 0.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = down[i], p2 = down[i+up.size()/2];
        glVertex3d(p1[0], p1[1], p1[2]);
        glVertex3d(p2[0], p2[1], p2[2]);
        glEnd();
    }
}
void UUV_MODEL::Draw_B(Isometry3d& pose){
    vector<Vector3d>point;
    point.push_back(pose.rotation()*V1.mid+pose.translation());
    point.push_back(pose.rotation()*V2.mid+pose.translation());
    point.push_back(pose.rotation()*V3.mid+pose.translation());
    point.push_back(pose.rotation()*V4.mid+pose.translation());
    point.push_back(pose.rotation()*V5.mid+pose.translation());
    point.push_back(pose.rotation()*V6.mid+pose.translation());
    point.push_back(pose.rotation()*V7.mid+pose.translation());
    point.push_back(pose.rotation()*V8.mid+pose.translation());
    glColor3f(0.0, 0.0, 0.0);
    for(int i=0;i<7;i++){
        for(int j=i+1;j<8;j++){
            glBegin(GL_LINES);
            glVertex3d(point[i][0], point[i][1], point[i][2]);
            glVertex3d(point[j][0], point[j][1], point[j][2]);
            glEnd();
        }
    }
}

void UUV_MODEL::Draw_UUV(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& poses){
    Isometry3d& pose=poses.back();
    ang_Slow*=ang_Slow0;
    Matrix3d ang_slow_transpose = ang_Slow.transpose();
    Matrix3d product = ang_Slow * ang_slow_transpose;
    ang_Slow.normalize();

    Vector_propeller V1=generate_V(pos1,ang1*ang_Slow);
    Vector_propeller V2=generate_V(pos2,ang2*ang_Slow);
    Vector_propeller V3=generate_V(pos3,ang3*ang_Slow);
    Vector_propeller V4=generate_V(pos4,ang4*ang_Slow);
    Vector_propeller V5=generate_V(pos5,ang5*ang_Slow);
    Vector_propeller V6=generate_V(pos6,ang6*ang_Slow);
    Vector_propeller V7=generate_V(pos7,ang7*ang_Slow);
    Vector_propeller V8=generate_V(pos8,ang8*ang_Slow);
    Draw_V(V1,pose);
    Draw_V(V2,pose);
    Draw_V(V3,pose);
    Draw_V(V4,pose);
    Draw_V(V5,pose);
    Draw_V(V6,pose);
    Draw_V(V7,pose);
    Draw_V(V8,pose);
    Draw_B(pose);
}
