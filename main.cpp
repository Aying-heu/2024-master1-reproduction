#include"A_include.h"

int main(){
    /*
    1.dynamics      Tau --> V_dot
    2.更新此时速度    V_next=V_now+V_dot*T

    3.kinematics    V --> Eta_dot
    4.更新此时位置    Eta_next=Eta_now+Eta_dot*T

    5.更新参数
        下一轮计算时，
        力   Tau  由控制器生成
        速度 V    由 dynamics 更新
        位置 Eta  由 kinematics 更新

        由于 Tau V Eta 自动更新，所以
        X,Y,Z,K,M,N
        u,v,w,p,q,r
        x,y,z,phi,theta,psi
        需要随着 Tau V Eta 的更新而更新

        其次，根据 dynamics 和 kinematics 需要更新的量有：
        CV DV g_Eta J(J_eta)

        将其在一轮计算后 一同更新 Update_Params()

    6.可视化
        可视化分为   实时  和  汇总
        需要        实时的 位置。  position_list
        可以的话    姿态，速度     pose_list  speed_list

    */


    PARAMS params;
    params.Calculate(0);    // 0 计算完在绘制  1 边计算边绘制

    // // 创建波浪可视化器
    // WaveVisualizer waveVis(1024, 768, "Wave Spectrum Visualization");
    
    // // 设置波浪谱参数
    // WaveSpectrumParams params;
    // params.type = JONSWAP;
    // params.windSpeed = 50.0f;  // 15 m/s
    // params.fetch = 200000.0f;  // 200 km
    // params.peakEnhancement = 3.3f;
    // params.gamma = 3.3f;
    
    // waveVis.setWaveSpectrum(params);
    
    // // 生成波浪分量 (频率和方向)
    // // 增加这两个参数可以提高波浪的细节，但会降低性能
    // int numFrequencies = 20;    // 频率分量数量
    // int numDirections = 12;     // 方向分量数量
    // float peakDirection = 0.0f; // 主波浪方向 (0表示从左到右)
    
    // waveVis.generateWaveComponents(numFrequencies, numDirections, peakDirection);
    
    // // 运行可视化
    // waveVis.run();

    // return 0;





    return 0;
}




// // // const int WIDTH = 800;
// // // const int HEIGHT = 600;
// // // const float X_MIN = -100.0f;
// // // const float X_MAX = 100.0f;
// // // const float Y_MIN = -100.0f;
// // // const float Y_MAX = 100.0f;
// // // const int NUM_POINTS = 100;

// // // // 计算单一方向的三维波动函数值
// // // float directionalWave(float x, float y, float t, float angle) {
// // //     const float g = 9.81f;
// // //     const float Ai = 5.1f;
// // //     const float wi = 2 * M_PI / 5.0f;
// // //     const float ki = wi * wi / g;

// // //     // 旋转坐标到波传播方向
// // //     float ca = cos(angle);
// // //     float sa = sin(angle);
// // //     float u = x * ca + y * sa;  // 沿传播方向的距离
    
// // //     // Stokes波二阶近似
// // //     float z = Ai * cos(wi * t - ki * u) + 0.5f * Ai * Ai * ki * pow(cos(wi * t - ki * u), 2);
// // //     return z;
// // // }

// // // // 计算两个方向波的叠加
// // // float waveFunction(float x, float y, float t) {
// // //     // 第一个波：沿x轴方向（0度）
// // //     float wave1 = directionalWave(x, y, t, 0.0f);
    
// // //     // 第二个波：45度方向
// // //     float wave2 = directionalWave(x, y, t, M_PI/4.0f);
    
// // //     // 返回两个波的叠加（振幅减半以避免过度放大）
// // //     return (wave1 + wave2) * 0.7f;
// // // }

// // // int main(int argc, char** argv) {
// // //     // 初始化Pangolin窗口
// // //     pangolin::CreateWindowAndBind("3D Wave Visualization", WIDTH, HEIGHT);
// // //     glEnable(GL_DEPTH_TEST);

// // //     // 创建视图
// // //     pangolin::OpenGlRenderState s_cam(
// // //         pangolin::ProjectionMatrix(WIDTH, HEIGHT, 420, 420, WIDTH/2, HEIGHT/2, 0.1, 1000),
// // //         pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
// // //     );

// // //     // 创建交互视图
// // //     pangolin::Handler3D handler(s_cam);
// // //     pangolin::View& d_cam = pangolin::CreateDisplay()
// // //         .SetBounds(0.0, 1.0, 0.0, 1.0, -WIDTH/(float)HEIGHT)
// // //         .SetHandler(&handler);

// // //     // 时间变量
// // //     float time = 0.0f;
// // //     const float timeStep = 0.03f;

// // //     // 预计算顶点位置
// // //     std::vector<std::vector<float>> vertices(NUM_POINTS, std::vector<float>(NUM_POINTS));
// // //     std::vector<std::vector<float>> colors(NUM_POINTS, std::vector<float>(NUM_POINTS));

// // //     while (!pangolin::ShouldQuit()) {
// // //         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// // //         d_cam.Activate(s_cam);

// // //         // 设置背景色
// // //         glClearColor(0.95f, 0.95f, 0.95f, 1.0f);

// // //         // 绘制网格
// // //         glColor3f(0.7f, 0.7f, 0.7f);
// // //         glLineWidth(1.0f);
// // //         glBegin(GL_LINES);
// // //         for (float x = X_MIN; x <= X_MAX; x += 20.0f) {
// // //             glVertex3f(x, Y_MIN, -5);
// // //             glVertex3f(x, Y_MAX, -5);
// // //         }
// // //         for (float y = Y_MIN; y <= Y_MAX; y += 20.0f) {
// // //             glVertex3f(X_MIN, y, -5);
// // //             glVertex3f(X_MAX, y, -5);
// // //         }
// // //         glEnd();

// // //         // 绘制坐标轴
// // //         glLineWidth(2.0f);
// // //         glColor3f(1.0f, 0.0f, 0.0f); // X轴 - 红色
// // //         glBegin(GL_LINES);
// // //         glVertex3f(X_MIN, 0, -5);
// // //         glVertex3f(X_MAX, 0, -5);
// // //         glEnd();

// // //         glColor3f(0.0f, 1.0f, 0.0f); // Y轴 - 绿色
// // //         glBegin(GL_LINES);
// // //         glVertex3f(0, Y_MIN, -5);
// // //         glVertex3f(0, Y_MAX, -5);
// // //         glEnd();

// // //         glColor3f(0.0f, 0.0f, 1.0f); // Z轴 - 蓝色
// // //         glBegin(GL_LINES);
// // //         glVertex3f(0, 0, -10);
// // //         glVertex3f(0, 0, 10);
// // //         glEnd();

// // //         // 计算当前时间的顶点高度和颜色
// // //         for (int i = 0; i < NUM_POINTS; ++i) {
// // //             for (int j = 0; j < NUM_POINTS; ++j) {
// // //                 float x = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// // //                 float y = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// // //                 vertices[i][j] = waveFunction(x, y, time);
                
// // //                 // 根据高度设置颜色 (蓝色到红色渐变)
// // //                 float heightNorm = (vertices[i][j] + 2.0f) / 4.0f;
// // //                 if (heightNorm < 0.0f) heightNorm = 0.0f;
// // //                 if (heightNorm > 1.0f) heightNorm = 1.0f;
// // //                 colors[i][j] = heightNorm;
// // //             }
// // //         }

// // //         // 绘制三维表面
// // //         glEnable(GL_BLEND);
// // //         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// // //         glLineWidth(1.5f);

// // //         // 绘制网格线
// // //         glColor3f(0.2f, 0.2f, 0.2f);
// // //         glBegin(GL_LINES);
// // //         for (int i = 0; i < NUM_POINTS; ++i) {
// // //             for (int j = 0; j < NUM_POINTS - 1; ++j) {
// // //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// // //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// // //                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
// // //                 glVertex3f(x1, y1, vertices[i][j]);
// // //                 glVertex3f(x1, y2, vertices[i][j + 1]);
// // //             }
// // //         }

// // //         for (int j = 0; j < NUM_POINTS; ++j) {
// // //             for (int i = 0; i < NUM_POINTS - 1; ++i) {
// // //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// // //                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
// // //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
                
// // //                 glVertex3f(x1, y1, vertices[i][j]);
// // //                 glVertex3f(x2, y1, vertices[i + 1][j]);
// // //             }
// // //         }
// // //         glEnd();

// // //         // 绘制三角形面
// // //         glBegin(GL_TRIANGLES);
// // //         for (int i = 0; i < NUM_POINTS - 1; ++i) {
// // //             for (int j = 0; j < NUM_POINTS - 1; ++j) {
// // //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// // //                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
// // //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// // //                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
// // //                 // 三角形1
// // //                 glColor4f(1.0f - colors[i][j], 0.0f, colors[i][j], 0.7f);
// // //                 glVertex3f(x1, y1, vertices[i][j]);
                
// // //                 glColor4f(1.0f - colors[i + 1][j], 0.0f, colors[i + 1][j], 0.7f);
// // //                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
// // //                 glColor4f(1.0f - colors[i][j + 1], 0.0f, colors[i][j + 1], 0.7f);
// // //                 glVertex3f(x1, y2, vertices[i][j + 1]);
                
// // //                 // 三角形2
// // //                 glColor4f(1.0f - colors[i + 1][j], 0.0f, colors[i + 1][j], 0.7f);
// // //                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
// // //                 glColor4f(1.0f - colors[i + 1][j + 1], 0.0f, colors[i + 1][j + 1], 0.7f);
// // //                 glVertex3f(x2, y2, vertices[i + 1][j + 1]);
                
// // //                 glColor4f(1.0f - colors[i][j + 1], 0.0f, colors[i][j + 1], 0.7f);
// // //                 glVertex3f(x1, y2, vertices[i][j + 1]);
// // //             }
// // //         }
// // //         glEnd();
// // //         glDisable(GL_BLEND);

// // //         // 更新时间
// // //         time += timeStep;

// // //         // 处理输入和渲染
// // //         pangolin::FinishFrame();
// // //     }

// // //     return 0;




// // const int WIDTH = 800;
// // const int HEIGHT = 600;
// // const float X_MIN = -50.0f;
// // const float X_MAX = 50.0f;
// // const float Y_MIN = -50.0f;
// // const float Y_MAX = 50.0f;
// // const int NUM_POINTS = 200;
// // const int NUM_DIRECTIONS = 10;  // 波浪方向数
// // const int NUM_WAVES = 50;       // 波浪数

// // // 波浪参数结构体
// // struct Wave {
// //     float amplitude;
// //     float frequency;
// //     float direction;  // 弧度
// //     float phase;
// // };

// // // JONSWAP波浪谱
// // float jonswapSpectrum(float freq, float peakFreq, float significantWaveHeight) {
// //     const float gamma = 3.3;  // JONSWAP谱峰增强因子
// //     const float g = 9.81;     // 重力加速度
    
// //     // Pierson-Moskowitz谱
// //     float pm = (5.0/16.0) * pow(significantWaveHeight, 2) * pow(peakFreq, 4) * 
// //                pow(freq, -5) * exp(-5.0/4.0 * pow(peakFreq/freq, 4));
    
// //     // JONSWAP修正因子
// //     float sigma = (freq <= peakFreq) ? 0.07 : 0.09;
// //     float gammaFactor = pow(gamma, exp(-pow(freq - peakFreq, 2) / (2 * sigma * sigma * peakFreq * peakFreq)));
    
// //     return pm * gammaFactor;
// // }

// // // 方向分布函数
// // float directionalDistribution(float direction, float meanDirection, float spreadingParameter) {
// //     const float pi = M_PI;
// //     float cosTerm = cos(direction - meanDirection);
// //     return (2.0 * spreadingParameter + 1.0) / (2.0 * pi) * 
// //            pow(cosTerm, 2 * spreadingParameter);
// // }

// // // 初始化波浪参数
// // std::vector<Wave> initializeWaves() {
// //     std::vector<Wave> waves;
// //     waves.reserve(NUM_WAVES);
    
// //     std::random_device rd;
// //     std::mt19937 gen(rd());
// //     std::uniform_real_distribution<> phaseDist(0, 2 * M_PI);
    
// //     // 波浪谱参数
// //     float significantWaveHeight = 4.0;  // 有效波高 (米)
// //     float peakPeriod = 5.0;             // 峰值周期 (秒)
// //     float peakFreq = 2.0 * M_PI / peakPeriod;
// //     float meanDirection = 0.0;          // 主波向 (弧度)
// //     float spreadingParameter = 10.0;    // 方向扩展参数
    
// //     // 频率范围
// //     float minFreq = peakFreq / 3.0;
// //     float maxFreq = peakFreq * 3.0;
// //     float freqStep = (maxFreq - minFreq) / (NUM_WAVES - 1);
    
// //     // 生成波浪
// //     for (int i = 0; i < NUM_WAVES; ++i) {
// //         float freq = minFreq + i * freqStep;
        
// //         // 从JONSWAP谱计算振幅
// //         float specValue = jonswapSpectrum(freq, peakFreq, significantWaveHeight);
// //         float amplitude = sqrt(2 * specValue * freqStep);
        
// //         // 从方向分布中选择方向
// //         float direction = 0.0;
// //         if (i < NUM_DIRECTIONS) {
// //             // 均匀分布在30个方向上
// //             direction = 2.0 * M_PI * i / NUM_DIRECTIONS;
// //         } else {
// //             // 随机方向，但偏向主波向
// //             std::normal_distribution<> dirDist(meanDirection, M_PI / 6.0);
// //             direction = fmod(dirDist(gen), 2.0 * M_PI);
// //         }
        
// //         // 随机相位
// //         float phase = phaseDist(gen);
        
// //         waves.push_back({amplitude, freq, direction, phase});
// //     }
    
// //     return waves;
// // }

// // // 计算基于波浪谱的三维波动函数值
// // float waveFunction(float x, float y, float t, const std::vector<Wave>& waves) {
// //     const float g = 9.81;
// //     float height = 0.0;
    
// //     for (const auto& wave : waves) {
// //         float k = wave.frequency * wave.frequency / g;  // 波数
// //         float kx = k * cos(wave.direction);
// //         float ky = k * sin(wave.direction);
// //         height += wave.amplitude * cos(wave.frequency * t - (kx * x + ky * y) + wave.phase);
// //     }
    
// //     return height;
// // }

// // int main(int argc, char** argv) {
// //     // 初始化波浪参数
// //     auto waves = initializeWaves();
    
// //     // 初始化Pangolin窗口
// //     pangolin::CreateWindowAndBind("Wave Spectrum Visualization", WIDTH, HEIGHT);
// //     glEnable(GL_DEPTH_TEST);

// //     // 创建视图
// //     pangolin::OpenGlRenderState s_cam(
// //         pangolin::ProjectionMatrix(WIDTH, HEIGHT, 420, 420, WIDTH/2, HEIGHT/2, 0.1, 1000),
// //         pangolin::ModelViewLookAt(300, 300, 300, 0, 0, 0, pangolin::AxisZ)
// //     );

// //     // 创建交互视图
// //     pangolin::Handler3D handler(s_cam);
// //     pangolin::View& d_cam = pangolin::CreateDisplay()
// //         .SetBounds(0.0, 1.0, 0.0, 1.0, -WIDTH/(float)HEIGHT)
// //         .SetHandler(&handler);

// //     // 时间变量
// //     float time = 0.0f;
// //     const float timeStep = 0.03f;

// //     // 预计算顶点位置
// //     std::vector<std::vector<float>> vertices(NUM_POINTS, std::vector<float>(NUM_POINTS));
// //     std::vector<std::vector<float>> colors(NUM_POINTS, std::vector<float>(NUM_POINTS));

// //     while (!pangolin::ShouldQuit()) {
// //         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// //         d_cam.Activate(s_cam);

// //         // 设置背景色为天空蓝
// //         glClearColor(0.95f, 0.95f, 0.95f, 1.0f);

// //         // 绘制网格
// //         glColor3f(0.7f, 0.7f, 0.7f);
// //         glLineWidth(1.0f);
// //         glBegin(GL_LINES);
// //         for (float x = X_MIN; x <= X_MAX; x += 10.0f) {
// //             glVertex3f(x, Y_MIN, 0);
// //             glVertex3f(x, Y_MAX, 0);
// //         }
// //         for (float y = Y_MIN; y <= Y_MAX; y += 10.0f) {
// //             glVertex3f(X_MIN, y, 0);
// //             glVertex3f(X_MAX, y, 0);
// //         }
// //         glEnd();

// //         // 绘制坐标轴
// //         glLineWidth(2.0f);
// //         glColor3f(1.0f, 0.0f, 0.0f); // X轴 - 红色
// //         glBegin(GL_LINES);
// //         glVertex3f(X_MIN, 0, 0);
// //         glVertex3f(X_MAX, 0, 0);
// //         glEnd();

// //         glColor3f(0.0f, 1.0f, 0.0f); // Y轴 - 绿色
// //         glBegin(GL_LINES);
// //         glVertex3f(0, Y_MIN, 0);
// //         glVertex3f(0, Y_MAX, 0);
// //         glEnd();

// //         glColor3f(0.0f, 0.0f, 1.0f); // Z轴 - 蓝色
// //         glBegin(GL_LINES);
// //         glVertex3f(0, 0, -20);
// //         glVertex3f(0, 0, 20);
// //         glEnd();

// //         // 计算当前时间的顶点高度和颜色
// //         float maxHeight = -10;
// //         float minHeight = 10;
        
// //         for (int i = 0; i < NUM_POINTS; ++i) {
// //             for (int j = 0; j < NUM_POINTS; ++j) {
// //                 float x = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// //                 float y = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// //                 vertices[i][j] = waveFunction(x, y, time, waves);
                
// //                 // 根据高度设置颜色 (蓝色到红色渐变)
// //                 float heightNorm = (vertices[i][j] + 2.0f) / 4.0f;
// //                 if (heightNorm < 0.0f) heightNorm = 0.0f;
// //                 if (heightNorm > 1.0f) heightNorm = 1.0f;
// //                 colors[i][j] = heightNorm;
// //             }
// //         }

// //         // 根据实际高度范围设置颜色
// //         for (int i = 0; i < NUM_POINTS; ++i) {
// //             for (int j = 0; j < NUM_POINTS; ++j) {
// //                 // 线性映射到[0,1]范围
// //                 colors[i][j] = (vertices[i][j] - minHeight) / (maxHeight - minHeight);
// //             }
// //         }

// //         // 绘制三维表面
// //         glEnable(GL_BLEND);
// //         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// //         glLineWidth(1.5f);

// //         // 绘制网格线
// //         glColor3f(0.2f, 0.2f, 0.2f);
// //         glBegin(GL_LINES);
// //         for (int i = 0; i < NUM_POINTS; ++i) {
// //             for (int j = 0; j < NUM_POINTS - 1; ++j) {
// //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// //                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
// //                 glVertex3f(x1, y1, vertices[i][j]);
// //                 glVertex3f(x1, y2, vertices[i][j + 1]);
// //             }
// //         }

// //         for (int j = 0; j < NUM_POINTS; ++j) {
// //             for (int i = 0; i < NUM_POINTS - 1; ++i) {
// //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// //                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
// //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
                
// //                 glVertex3f(x1, y1, vertices[i][j]);
// //                 glVertex3f(x2, y1, vertices[i + 1][j]);
// //             }
// //         }
// //         glEnd();

// //         // 绘制三角形面
// //         glBegin(GL_TRIANGLES);
// //         for (int i = 0; i < NUM_POINTS - 1; ++i) {
// //             for (int j = 0; j < NUM_POINTS - 1; ++j) {
// //                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
// //                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
// //                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
// //                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
// //                 // 三角形1
// //                 glColor4f(1.0f - colors[i][j], 0.0f, colors[i][j], 0.7f);
// //                 glVertex3f(x1, y1, vertices[i][j]);
                
// //                 glColor4f(1.0f - colors[i + 1][j], 0.0f, colors[i + 1][j], 0.7f);
// //                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
// //                 glColor4f(1.0f - colors[i][j + 1], 0.0f, colors[i][j + 1], 0.7f);
// //                 glVertex3f(x1, y2, vertices[i][j + 1]);
                
// //                 // 三角形2
// //                 glColor4f(1.0f - colors[i + 1][j], 0.0f, colors[i + 1][j], 0.7f);
// //                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
// //                 glColor4f(1.0f - colors[i + 1][j + 1], 0.0f, colors[i + 1][j + 1], 0.7f);
// //                 glVertex3f(x2, y2, vertices[i + 1][j + 1]);
                
// //                 glColor4f(1.0f - colors[i][j + 1], 0.0f, colors[i][j + 1], 0.7f);
// //                 glVertex3f(x1, y2, vertices[i][j + 1]);
// //             }
// //         }
// //         glEnd();
// //         glDisable(GL_BLEND);

// //         // 更新时间
// //         time += timeStep;

// //         // 处理输入和渲染
// //         pangolin::FinishFrame();
// //     }

// //     return 0;
// // }    













// #include<A_include.h>
// #include <pangolin/pangolin.h>
// #include <cmath>
// #include <vector>
// #include <random>
// #include <algorithm>
// #include <Eigen/Dense>
// #include <cstdlib> // 添加环境变量支持

// const int WIDTH = 800;
// const int HEIGHT = 600;
// const float X_MIN = -30.0f;
// const float X_MAX = 30.0f;
// const float Y_MIN = -30.0f;
// const float Y_MAX = 30.0f;
// const int NUM_POINTS = 80;

// // 波浪分量结构体
// struct WaveComponent {
//     float amplitude;    // 振幅
//     float frequency;    // 频率 (rad/s)
//     float waveNumber;   // 波数
//     float phase;        // 初始相位
//     float direction;    // 传播方向 (radians)
// };

// // 波浪谱参数
// const int NUM_DIRECTIONS = 10;  // 方向数量
// const int NUM_FREQUENCIES = 50; // 频率数量
// const float WAVE_HEIGHT = 4.0f; // 有效波高 (m)
// const float PEAK_PERIOD = 5.0f; // 峰值周期 (s)
// const float GRAVITY = 9.81f;    // 重力加速度

// // 生成JONSWAP波浪谱
// std::vector<WaveComponent> generateWaveSpectrum() {
//     std::vector<WaveComponent> components;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<float> phaseDist(0.0f, 2.0f * M_PI);
    
//     // 计算峰值频率
//     const float omega_p = 2.0f * M_PI / PEAK_PERIOD;
    
//     // 频率范围 (0.1倍峰值频率 到 3倍峰值频率)
//     const float minFreq = omega_p * 0.1f;
//     const float maxFreq = omega_p * 3.0f;
//     const float df = (maxFreq - minFreq) / NUM_FREQUENCIES;
    
//     // 方向范围 (0 到 2π)
//     const float dd = 2.0f * M_PI / NUM_DIRECTIONS;
    
//     // 总能量
//     float totalEnergy = 0.0f;
    
//     // 生成频率和方向网格
//     for (int i = 0; i < NUM_FREQUENCIES; ++i) {
//         float omega = minFreq + i * df;
        
//         for (int j = 0; j < NUM_DIRECTIONS; ++j) {
//             float theta = j * dd;
            
//             // JONSWAP谱模型
//             float sigma = omega <= omega_p ? 0.07f : 0.09f;
//             float r = exp(-(pow(omega - omega_p, 2) / (2.0f * pow(sigma, 2) * pow(omega_p, 2))));
//             float gamma = 3.3f; // 峰值增强因子
            
//             float S = (5.0f/16.0f) * pow(WAVE_HEIGHT, 2) * pow(omega_p, 4) * pow(omega, -5) * 
//                       exp(-1.25f * pow(omega_p/omega, 4)) * pow(gamma, r);
            
//             // 方向分布函数 (cosine-squared)
//             float D = (2.0f/M_PI) * pow(cos(theta), 2);
            
//             // 能量密度
//             float energyDensity = S * D * df * dd;
            
//             // 计算振幅 (a = √(2 * S(ω) * dω * dθ))
//             float amplitude = sqrt(2.0f * energyDensity);
            
//             // 波数 (深水波色散关系)
//             float k = pow(omega, 2) / GRAVITY;
            
//             // 随机相位
//             float phase = phaseDist(gen);
            
//             components.push_back({amplitude, omega, k, phase, theta});
//             totalEnergy += 0.5f * pow(amplitude, 2);
//         }
//     }
    
//     // 归一化振幅以满足目标波高
//     float actualHs = 4.0f * sqrt(totalEnergy);
//     float scale = WAVE_HEIGHT / actualHs;
    
//     for (auto& comp : components) {
//         comp.amplitude *= scale;
//     }
    
//     return components;
// }

// // 全局波浪分量
// std::vector<WaveComponent> waveComponents;

// // 计算波浪高度
// float waveFunction(float x, float y, float t) {
//     float height = 0.0f;
    
//     for (const auto& comp : waveComponents) {
//         // 方向向量
//         float dx = cos(comp.direction);
//         float dy = sin(comp.direction);
        
//         // 波浪传播方向上的位置
//         float propagation = x * dx + y * dy;
        
//         // 波浪高度贡献
//         height += comp.amplitude * cos(comp.waveNumber * propagation - comp.frequency * t + comp.phase);
//     }
    
//     return height;
// }

// int main(int argc, char** argv) {

//      // 修复Wayland兼容性问题
//     setenv("GDK_BACKEND", "x11", 1);
    

    
//     // 生成波浪谱
//     waveComponents = generateWaveSpectrum();
//     std::cout << "Generated " << waveComponents.size() << " wave components." << std::endl;
    
//     // 初始化Pangolin窗口
//     pangolin::CreateWindowAndBind("UUV Wave Dynamics Simulation", WIDTH, HEIGHT);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
//     // 创建视图
//     pangolin::OpenGlRenderState s_cam(
//         pangolin::ProjectionMatrix(WIDTH, HEIGHT, 420, 420, WIDTH/2, HEIGHT/2, 0.1, 1000),
//         pangolin::ModelViewLookAt(0, -200, 100, 0, 0, 0, pangolin::AxisZ)
//     );

//     // 创建交互视图
//     pangolin::Handler3D handler(s_cam);
//     pangolin::View& d_cam = pangolin::CreateDisplay()
//         .SetBounds(0.0, 1.0, 0.0, 1.0, -WIDTH/(float)HEIGHT)
//         .SetHandler(&handler);

//     // 时间变量
//     float time = 0.0f;
//     const float timeStep = 0.03f;

//     // 预计算顶点位置
//     std::vector<std::vector<float>> vertices(NUM_POINTS, std::vector<float>(NUM_POINTS));
//     std::vector<std::vector<float>> colors(NUM_POINTS, std::vector<float>(NUM_POINTS));

//     // 波浪统计
//     float maxWaveHeight = 0.0f;
//     float minWaveHeight = 0.0f;

//     while (!pangolin::ShouldQuit()) {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);

//         // 设置背景色
//         glClearColor(0.1f, 0.1f, 0.2f, 1.0f);

//         // 绘制坐标轴
//         glLineWidth(2.0f);
//         glColor3f(1.0f, 0.0f, 0.0f); // X轴 - 红色
//         glBegin(GL_LINES);
//         glVertex3f(X_MIN, 0, -10);
//         glVertex3f(X_MAX, 0, -10);
//         glEnd();

//         glColor3f(0.0f, 1.0f, 0.0f); // Y轴 - 绿色
//         glBegin(GL_LINES);
//         glVertex3f(0, Y_MIN, -10);
//         glVertex3f(0, Y_MAX, -10);
//         glEnd();

//         glColor3f(0.0f, 0.0f, 1.0f); // Z轴 - 蓝色
//         glBegin(GL_LINES);
//         glVertex3f(0, 0, -15);
//         glVertex3f(0, 0, 15);
//         glEnd();

//         // 计算当前时间的顶点高度和颜色
//         maxWaveHeight = -100.0f;
//         minWaveHeight = 100.0f;
        
//         for (int i = 0; i < NUM_POINTS; ++i) {
//             for (int j = 0; j < NUM_POINTS; ++j) {
//                 float x = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//                 float y = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//                 vertices[i][j] = waveFunction(x, y, time);
                
//                 // 更新波浪高度统计
//                 if (vertices[i][j] > maxWaveHeight) maxWaveHeight = vertices[i][j];
//                 if (vertices[i][j] < minWaveHeight) minWaveHeight = vertices[i][j];
//             }
//         }
        
//         // 计算颜色
//         float range = maxWaveHeight - minWaveHeight;
//         if (range < 0.01f) range = 0.01f; // 避免除以零
        
//         for (int i = 0; i < NUM_POINTS; ++i) {
//             for (int j = 0; j < NUM_POINTS; ++j) {
//                 // 根据高度设置颜色 (深蓝到浅蓝渐变)
//                 float heightNorm = (vertices[i][j] - minWaveHeight) / range;
//                 colors[i][j] = heightNorm;
//             }
//         }

//         // 绘制三维表面
//         glEnable(GL_BLEND);
//         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//         glLineWidth(1.0f);

//         // 绘制网格线
//         glColor4f(0.5f, 0.7f, 1.0f, 0.3f);
//         glBegin(GL_LINES);
//         for (int i = 0; i < NUM_POINTS; i += 2) {
//             for (int j = 0; j < NUM_POINTS - 1; j += 2) {
//                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
//                 glVertex3f(x1, y1, vertices[i][j]);
//                 glVertex3f(x1, y2, vertices[i][j + 1]);
//             }
//         }

//         for (int j = 0; j < NUM_POINTS; j += 2) {
//             for (int i = 0; i < NUM_POINTS - 1; i += 2) {
//                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
//                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
                
//                 glVertex3f(x1, y1, vertices[i][j]);
//                 glVertex3f(x2, y1, vertices[i + 1][j]);
//             }
//         }
//         glEnd();

//         // 绘制三角形面
//         glBegin(GL_TRIANGLES);
//         for (int i = 0; i < NUM_POINTS - 1; ++i) {
//             for (int j = 0; j < NUM_POINTS - 1; ++j) {
//                 float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//                 float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
//                 float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//                 float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
                
//                 // 计算颜色 (基于高度)
//                 float c1 = colors[i][j];
//                 float c2 = colors[i + 1][j];
//                 float c3 = colors[i][j + 1];
//                 float c4 = colors[i + 1][j + 1];
                
//                 // 三角形1
//                 glColor4f(0.2f, 0.3f, 0.8f, c1 * 0.7f + 0.3f);
//                 glVertex3f(x1, y1, vertices[i][j]);
                
//                 glColor4f(0.2f, 0.3f, 0.8f, c2 * 0.7f + 0.3f);
//                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
//                 glColor4f(0.2f, 0.3f, 0.8f, c3 * 0.7f + 0.3f);
//                 glVertex3f(x1, y2, vertices[i][j + 1]);
                
//                 // 三角形2
//                 glColor4f(0.2f, 0.3f, 0.8f, c2 * 0.7f + 0.3f);
//                 glVertex3f(x2, y1, vertices[i + 1][j]);
                
//                 glColor4f(0.2f, 0.5f, 1.0f, c4 * 0.7f + 0.3f);
//                 glVertex3f(x2, y2, vertices[i + 1][j + 1]);
                
//                 glColor4f(0.2f, 0.3f, 0.8f, c3 * 0.7f + 0.3f);
//                 glVertex3f(x1, y2, vertices[i][j + 1]);
//             }
//         }
//         glEnd();
//         glDisable(GL_BLEND);

//         // 显示波浪信息
//         // glColor3f(1.0f, 1.0f, 1.0f);
//         // pangolin::GlFont::I().Text("Wave Spectrum Visualization").Draw(5, 30);
//         // pangolin::GlFont::I().Text("Components: %d directions x %d frequencies = %d waves", 
//         //                           NUM_DIRECTIONS, NUM_FREQUENCIES, NUM_DIRECTIONS*NUM_FREQUENCIES).Draw(5, 55);
//         // pangolin::GlFont::I().Text("Wave Height: %.1fm (min: %.2fm, max: %.2fm)", 
//         //                           WAVE_HEIGHT, minWaveHeight, maxWaveHeight).Draw(5, 80);
//         // pangolin::GlFont::I().Text("Peak Period: %.1fs", PEAK_PERIOD).Draw(5, 105);
//         // pangolin::GlFont::I().Text("Time: %.1fs", time).Draw(5, 130);

//         // 更新时间
//         time += timeStep;

//         // 处理输入和渲染
//         pangolin::FinishFrame();
//     }

//     return 0;
// }



// #include<A_include.h>
// #include <pangolin/pangolin.h>
// #include <cmath>
// #include <vector>
// #include <iostream>
// #include "wave.h"

// const int WIDTH = 800;
// const int HEIGHT = 600;
// const float X_MIN = -30.0f;
// const float X_MAX = 30.0f;
// const float Y_MIN = -30.0f;
// const float Y_MAX = 30.0f;
// const int NUM_POINTS = 80;

// // 初始化Pangolin窗口
// pangolin::OpenGlRenderState initializePangolin() {
//     pangolin::CreateWindowAndBind("UUV Wave Dynamics Simulation", WIDTH, HEIGHT);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
//     // 创建视图
//     return pangolin::OpenGlRenderState(
//         pangolin::ProjectionMatrix(WIDTH, HEIGHT, 420, 420, WIDTH/2, HEIGHT/2, 0.1, 1000),
//         pangolin::ModelViewLookAt(0, -200, 100, 0, 0, 0, pangolin::AxisZ)
//     );
// }

// // 绘制坐标轴
// void drawAxes() {
//     glLineWidth(2.0f);
    
//     glColor3f(1.0f, 0.0f, 0.0f); // X轴 - 红色
//     glBegin(GL_LINES);
//     glVertex3f(X_MIN, 0, -10);
//     glVertex3f(X_MAX, 0, -10);
//     glEnd();

//     glColor3f(0.0f, 1.0f, 0.0f); // Y轴 - 绿色
//     glBegin(GL_LINES);
//     glVertex3f(0, Y_MIN, -10);
//     glVertex3f(0, Y_MAX, -10);
//     glEnd();

//     glColor3f(0.0f, 0.0f, 1.0f); // Z轴 - 蓝色
//     glBegin(GL_LINES);
//     glVertex3f(0, 0, -15);
//     glVertex3f(0, 0, 15);
//     glEnd();
// }

// // 计算波浪表面的顶点高度和颜色
// void calculateWaveSurface(const WaveSpectrum& spectrum, 
//                          std::vector<std::vector<float>>& vertices,
//                          std::vector<std::vector<float>>& colors,
//                          float time, float& maxHeight, float& minHeight) {
//     maxHeight = -100.0f;
//     minHeight = 100.0f;
    
//     for (int i = 0; i < NUM_POINTS; ++i) {
//         for (int j = 0; j < NUM_POINTS; ++j) {
//             float x = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//             float y = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//             vertices[i][j] = calculateWaveHeight(spectrum, x, y, time);
            
//             // 更新波浪高度统计
//             if (vertices[i][j] > maxHeight) maxHeight = vertices[i][j];
//             if (vertices[i][j] < minHeight) minHeight = vertices[i][j];
//         }
//     }
    
//     // 计算颜色
//     float range = maxHeight - minHeight;
//     if (range < 0.01f) range = 0.01f; // 避免除以零
    
//     for (int i = 0; i < NUM_POINTS; ++i) {
//         for (int j = 0; j < NUM_POINTS; ++j) {
//             // 根据高度设置颜色 (深蓝到浅蓝渐变)
//             float heightNorm = (vertices[i][j] - minHeight) / range;
//             colors[i][j] = heightNorm;
//         }
//     }
// }

// // 绘制波浪表面
// void drawWaveSurface(const std::vector<std::vector<float>>& vertices,
//                     const std::vector<std::vector<float>>& colors) {
//     // 绘制网格线
//     glColor4f(0.5f, 0.7f, 1.0f, 0.3f);
//     glBegin(GL_LINES);
//     for (int i = 0; i < NUM_POINTS; i += 2) {
//         for (int j = 0; j < NUM_POINTS - 1; j += 2) {
//             float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//             float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//             float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
            
//             glVertex3f(x1, y1, vertices[i][j]);
//             glVertex3f(x1, y2, vertices[i][j + 1]);
//         }
//     }

//     for (int j = 0; j < NUM_POINTS; j += 2) {
//         for (int i = 0; i < NUM_POINTS - 1; i += 2) {
//             float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//             float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
//             float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
            
//             glVertex3f(x1, y1, vertices[i][j]);
//             glVertex3f(x2, y1, vertices[i + 1][j]);
//         }
//     }
//     glEnd();

//     // 绘制三角形面
//     glBegin(GL_TRIANGLES);
//     for (int i = 0; i < NUM_POINTS - 1; ++i) {
//         for (int j = 0; j < NUM_POINTS - 1; ++j) {
//             float x1 = X_MIN + (X_MAX - X_MIN) * i / (NUM_POINTS - 1);
//             float x2 = X_MIN + (X_MAX - X_MIN) * (i + 1) / (NUM_POINTS - 1);
//             float y1 = Y_MIN + (Y_MAX - Y_MIN) * j / (NUM_POINTS - 1);
//             float y2 = Y_MIN + (Y_MAX - Y_MIN) * (j + 1) / (NUM_POINTS - 1);
            
//             // 计算颜色 (基于高度)
//             float c1 = colors[i][j];
//             float c2 = colors[i + 1][j];
//             float c3 = colors[i][j + 1];
//             float c4 = colors[i + 1][j + 1];
            
//             // 三角形1
//             glColor4f(0.2f, 0.3f, 0.8f, c1 * 0.7f + 0.3f);
//             glVertex3f(x1, y1, vertices[i][j]);
            
//             glColor4f(0.2f, 0.3f, 0.8f, c2 * 0.7f + 0.3f);
//             glVertex3f(x2, y1, vertices[i + 1][j]);
            
//             glColor4f(0.2f, 0.3f, 0.8f, c3 * 0.7f + 0.3f);
//             glVertex3f(x1, y2, vertices[i][j + 1]);
            
//             // 三角形2
//             glColor4f(0.2f, 0.3f, 0.8f, c2 * 0.7f + 0.3f);
//             glVertex3f(x2, y1, vertices[i + 1][j]);
            
//             glColor4f(0.2f, 0.5f, 1.0f, c4 * 0.7f + 0.3f);
//             glVertex3f(x2, y2, vertices[i + 1][j + 1]);
            
//             glColor4f(0.2f, 0.3f, 0.8f, c3 * 0.7f + 0.3f);
//             glVertex3f(x1, y2, vertices[i][j + 1]);
//         }
//     }
//     glEnd();
// }

// int main(int argc, char** argv) {
//     // 修复Wayland兼容性问题
//     // setenv("GDK_BACKEND", "x11", 1);
    
//     // 生成波浪谱
//     Wave wave;
//     std::cout<<wave.getWaveInfo()[-1].theta<<std::endl;

//     WaveSpectrumParams params;
//     params.waveHeight = 4.0f;
//     params.peakPeriod = 5.0f;
//     WaveSpectrum waveSpectrum = generateWaveSpectrum(params);
//     std::cout << "Generated " << waveSpectrum.components.size() << " wave components." << std::endl;
    
//     // 初始化Pangolin
//     pangolin::OpenGlRenderState s_cam = initializePangolin();

//     // 创建交互视图
//     pangolin::Handler3D handler(s_cam);
//     pangolin::View& d_cam = pangolin::CreateDisplay()
//         .SetBounds(0.0, 1.0, 0.0, 1.0, -WIDTH/(float)HEIGHT)
//         .SetHandler(&handler);

//     // 时间变量
//     float time = 0.0f;
//     const float timeStep = 0.03f;

//     // 预计算顶点位置和颜色
//     std::vector<std::vector<float>> vertices(NUM_POINTS, std::vector<float>(NUM_POINTS));
//     std::vector<std::vector<float>> colors(NUM_POINTS, std::vector<float>(NUM_POINTS));

//     // 波浪统计
//     float maxWaveHeight = 0.0f;
//     float minWaveHeight = 0.0f;

//     while (!pangolin::ShouldQuit()) {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         d_cam.Activate(s_cam);

//         // 设置背景色
//         glClearColor(0.1f, 0.1f, 0.2f, 1.0f);

//         // 绘制坐标轴
//         drawAxes();

//         // 计算当前时间的顶点高度和颜色
//         calculateWaveSurface(waveSpectrum, vertices, colors, time, maxWaveHeight, minWaveHeight);

//         // 绘制波浪表面
//         glEnable(GL_BLEND);
//         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//         drawWaveSurface(vertices, colors);
//         glDisable(GL_BLEND);

//         // 更新时间
//         time += timeStep;

//         // 处理输入和渲染
//         pangolin::FinishFrame();
//     }

//     return 0;
// }

