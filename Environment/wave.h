#ifndef WAVE_H
#define WAVE_H

#include<A_include.h>
#include <vector>
#include <random>
#include <Eigen/Dense>


// 单个波浪分量
struct Single_Wave {
    float amplitude;    // 振幅（浪高的一半）
    float omega;    // 频率 (rad/s)
    float phase;        // 初始相位 (rad)
    float theta;    // 传播方向 (rad)
};


// 波浪类，包含多个波浪分量及相关操作
class Wave {
public:
    // 构造函数
    Wave();

    float Hs;   // 峰值频率
    float T0;   // 峰值周期
    float T1;
    float gravity;

    float omega_max;
    float omega_min;
    float d_omega;
    float theta_max;
    float theta_min;
    float d_theta;
    std::vector<float> all_omega;
    std::vector<float> all_theta;
    // 存放波浪信息
    std::vector<Single_Wave> Wave_Info;

    void initializeWaveComponents();
    std::vector<Single_Wave>& getWaveInfo();

    // JONSWAP波浪谱；输入w和ttheta，输出S(wi,theta_i)
    float get_Spectrum(float omega,float theta);

    // 计算某点某时刻的波高
    float calculateHeight(float x, float y, float t);

};

#endif // WAVE_H
