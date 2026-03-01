#include<A_include.h>
Wave::Wave(){

    Hs=5.0;   // 峰值频率
    T0=4.0;   // 峰值周期
    T1=0.834*T0;
    gravity=9.81;

    omega_max=10.0;
    omega_min=0.1;
    d_omega=(omega_max-omega_min)/19;
    theta_max=2*M_PI;
    theta_min=0.0;
    d_theta=(theta_max-theta_min)/3;

    for (float i=0.1;i<=10;i+=d_omega){
        all_omega.push_back(i);
    }
    for (float i=0;i<=2*M_PI;i+=d_theta){
        cout<<i<<endl;
        all_theta.push_back(i);
    }

    // 初始化波浪信息数组
    initializeWaveComponents();
}

// 初始化波浪组成部分
void Wave::initializeWaveComponents() {
    Wave_Info.clear();
    for (float omega : all_omega) {
        for (float theta : all_theta) {
            Single_Wave wave;
            wave.omega = omega;
            wave.theta = theta;
            
            // 计算波浪振幅
            float spectrum = get_Spectrum(omega, theta);
            wave.amplitude = std::sqrt(2 * spectrum * d_omega * d_theta);
            
            // 随机相位
            wave.phase = (float)rand() / RAND_MAX * 2 * M_PI;
            
            Wave_Info.push_back(wave);
        }
    }
}


// handbook:
// S(w)= 155 * Hs^2 / T1^4 / w^5 * e^{ -944 / T1^4 /w^4 } * gamma^Y
// Y=e^[ -((0.191 * w * T1 -1)/(sqrt(2) * sigma))^2 ]
// sigma= w<=5.24/T1?0.07:0.09
// T1=0.834T0
// T0=2*M_PI / w0

// D(theta)=2/M_PI*cos(theta)^2   -pi/2<=theta<=pi/2   其他为0


float Wave::get_Spectrum(float omega,float theta){
    float gamma=3.3;
    float omega0=2*M_PI/T0;
    float sigma = (omega <= 5.24/T1) ? 0.07 : 0.09;
    float Y = std::exp(-std::pow((0.191 * omega * T1 - 1) / (std::sqrt(2) * sigma), 2));
    float S = 155 * std::pow(Hs, 2) / std::pow(T1, 4) / std::pow(omega, 5) * 
                    std::exp(-944 / std::pow(T1, 4) / std::pow(omega, 4)) * 
                    std::pow(gamma, Y);
    while (theta > M_PI) theta -= 2 * M_PI;
    while (theta < -M_PI) theta += 2 * M_PI;
    float D = std::abs(theta) <= M_PI/2 ? 2/M_PI * std::pow(std::cos(theta), 2) : 0.0;

    // 返回联合频谱
    return S * D;
}

// 计算某点某时刻的波高
// 计算某点某时刻的波高
float Wave::calculateHeight(float x, float y, float t) {
    float height = 0.0;
    
    // 假设水深足够大，使用深水色散关系
    float h = 1000.0; // 深水深度，单位：米
    
    for (const auto& wave : Wave_Info) {
        // 计算波数k，使用色散关系 ω² = g·k·tanh(k·h)
        // 对于深水情况，可以简化为 k = ω²/g
        float k = std::pow(wave.omega, 2) / gravity;
        
        // 计算波在x和y方向上的分量
        float kx = k * std::cos(wave.theta);
        float ky = k * std::sin(wave.theta);
        
        // 计算相位
        float phase = kx * x + ky * y - wave.omega * t + wave.phase;
        
        // 叠加波浪分量
        height += wave.amplitude * std::cos(phase);
    }
    
    return height;
}


std::vector<Single_Wave>& Wave::getWaveInfo()  {
    return Wave_Info;
}
