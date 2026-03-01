#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include<cmath>
#include<math.h>


#include <unistd.h>

#include <qpOASES.hpp>
// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>  // 包含特征值计算的头文件
#include <Eigen/Geometry>


using namespace Eigen;

#include <pangolin/pangolin.h>
#include <iomanip>

using namespace std;
#define PI acos(-1.0)


#include"dynamics.h"
#include"kinematics.h"

#include"model_UUV.h"
#include"Control.h"
#include"visualize.h"
#include"params.h"

#include"wave.h"

#include <cmath>
#include <random>



/*
#include"Draw_UUV.h"
#include"model_UUV.h"
*/
