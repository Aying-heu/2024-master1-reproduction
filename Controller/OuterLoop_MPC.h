// MPC.h
#pragma once
#include "../A_include.h"

class MPC {
public:
MPC();
    Eigen::Matrix<double, 6, 1> ComputeDesiredVelocity(
        double t,
        double T,
        double t_max,
        const Eigen::Matrix<double, 6, 1>& current_Eta,
        const Eigen::Matrix<double, 6, 1>& current_V,
        const vector<Eigen::Matrix<double, 5, 1>>& reference_Eta);
        void solveQP(const MatrixXd& H, const MatrixXd& g, const MatrixXd& A_I0, const MatrixXd& V_max, const MatrixXd& V_min,
                const MatrixXd& U_max, const MatrixXd& U_min,  VectorXd& U_star);
        MatrixXd Msqrt(MatrixXd A,int n);
        Eigen::MatrixXd kroneckerProduct(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

private:
    // MPC参数
    int prediction_horizon_Np;  // 预测时域
    int control_horizon_Nc;     // 控制时域
    Eigen::MatrixXd Q;       // 输出信号的加权矩阵
    Eigen::MatrixXd R;       // 控制信号的加权矩阵
    Matrix<double,5,1> V_max;
    Matrix<double,5,1> V_min;
    Matrix<double,5,1> U_max;
    Matrix<double,5,1> U_min;

    Matrix<double,5,5>I2;
    Matrix<double,5,5>O;
    Matrix<double,5,5>I;

    MatrixXd INp;
    MatrixXd INc;
    MatrixXd QQ ;
    MatrixXd RR ;
    MatrixXd A_I;
    MatrixXd V_T;
    MatrixXd V_max0;
    MatrixXd V_min0;
    MatrixXd U_max0;
    MatrixXd U_min0;



    Matrix<double,5,1>Eta;
    Matrix<double,5,1>V;
    Matrix<double,5,1>Eta_d;
    Matrix<double,5,1>V_d;
    double u_d,v_d,w_d,theta_d,psi_d;
    double a14,a15,a24,a25,a34,b11,b12,b13,b21,b22,b23,b31,b33,b55;
    Matrix<double,5,5>a;
    Matrix<double,5,5>b;
    Matrix<double,5,5>al;
    Matrix<double,5,5>bl;
    Matrix<double,10,10>A;
    Matrix<double,10,5>B;
    Matrix<double,5,10>C;
    MatrixXd E;
    MatrixXd H;
    MatrixXd g;
    Matrix<double,6,1> U_star0;

    void update_outloop_params(int prediction_horizon_Np,int control_horizon_Nc);
};


/*
 * 1. 假设控制输入序列

    初始假设：

        在k时刻，假设一个初始的控制输入序列 u(0),u(1),…,u(Nc−1)u(0),u(1),…,u(Nc​−1)。

        通常可以假设为全零序列（即 u(0)=0,u(1)=0,…,u(Nc−1)=0u(0)=0,u(1)=0,…,u(Nc​−1)=0），或者基于经验或其他启发式方法给出一个合理的初始值。

    假设的作用：

        假设的控制输入序列用于预测未来 NpNp​ 步的状态 x(1∣0),x(2∣0),…,x(Np∣0)x(1∣0),x(2∣0),…,x(Np​∣0)。

        这些预测的状态用于计算与参考轨迹的误差。

2. 预测未来状态

    预测模型：

        使用系统模型（如公式37-40）预测未来 NpNp​ 步的状态：
        x(k+1∣k)=Ax(k)+Bu(k)x(k+1∣k)=Ax(k)+Bu(k)x(k+2∣k)=Ax(k+1∣k)+Bu(k+1)=A2x(k)+ABu(k)+Bu(k+1)x(k+2∣k)=Ax(k+1∣k)+Bu(k+1)=A2x(k)+ABu(k)+Bu(k+1)⋮⋮x(k+Np∣k)=ANpx(k)+∑i=0Np−1AiBu(k+i)x(k+Np​∣k)=ANp​x(k)+∑i=0Np​−1​AiBu(k+i)

    预测的作用：

        预测的状态 x(1∣0),x(2∣0),…,x(Np∣0)x(1∣0),x(2∣0),…,x(Np​∣0) 用于计算与参考轨迹的误差。

3. 优化问题求解

    优化目标函数：

        定义优化目标函数，通常包括输出误差和控制输入的惩罚：
        min⁡J(k)=∑i=1Np∣∣y(k+i∣k)−yr(k+i)∣∣Q2+∑i=0Nc−1∣∣u(k+i)∣∣R2minJ(k)=∑i=1Np​​∣∣y(k+i∣k)−yr​(k+i)∣∣Q2​+∑i=0Nc​−1​∣∣u(k+i)∣∣R2​

        其中：

            y(k+i∣k)y(k+i∣k)：预测的输出（如位置和速度）。

            yr(k+i)yr​(k+i)：参考输出（如期望轨迹）。

            u(k+i)u(k+i)：控制输入序列（决策变量）。

            QQ 和 RR：权重矩阵，分别用于惩罚输出误差和控制输入。

    优化求解的结果：

        优化求解的结果是最优的控制输入序列 U=[u(0),u(1),…,u(Nc−1)]TU=[u(0),u(1),…,u(Nc​−1)]T。

        这个最优控制输入序列是使目标函数 J(k)J(k) 最小化的控制输入序列。

4. 优化求解出来的东西

    优化求解的结果：

        优化求解出来的东西是最优的控制输入序列 U=[u(0),u(1),…,u(Nc−1)]TU=[u(0),u(1),…,u(Nc​−1)]T。

        这个控制输入序列是使预测的未来状态 x(1∣0),x(2∣0),…,x(Np∣0)x(1∣0),x(2∣0),…,x(Np​∣0) 与参考轨迹 yr(1),yr(2),…,yr(Np)yr​(1),yr​(2),…,yr​(Np​) 的误差最小化的控制输入序列。

5. 是否需要重新预测？

    不需要重新预测：

        优化求解出来的控制输入序列 UU 已经是最优的，可以直接用于控制。

        在k时刻，只需要将最优控制序列的第一个元素 u(0)u(0) 应用于系统。

    滚动优化：

        在k+1时刻，重新进行预测和优化，得到新的控制输入序列 U=[u(1),u(2),…,u(Nc)]TU=[u(1),u(2),…,u(Nc​)]T。

        这种方法称为滚动优化，即在每个时间步重新优化控制输入序列。

6. 示例

假设在k=0时刻：

    测量当前状态：

        x(0)=[x0,y0,z0,φ0,θ0,ψ0]Tx(0)=[x0​,y0​,z0​,φ0​,θ0​,ψ0​]T。

    假设控制输入序列：

        假设一个初始的控制输入序列 u(0),u(1),…,u(Nc−1)u(0),u(1),…,u(Nc​−1)。

    预测未来状态：

        使用系统模型预测未来 NpNp​ 步的状态 x(1∣0),x(2∣0),…,x(Np∣0)x(1∣0),x(2∣0),…,x(Np​∣0)。

    优化问题求解：

        定义优化目标函数：
        min⁡J(0)=∑i=1Np∣∣y(i∣0)−yr(i)∣∣Q2+∑i=0Nc−1∣∣u(i)∣∣R2minJ(0)=∑i=1Np​​∣∣y(i∣0)−yr​(i)∣∣Q2​+∑i=0Nc​−1​∣∣u(i)∣∣R2​

        通过优化求解，找到最优控制输入序列 U=[u(0),u(1),…,u(Nc−1)]TU=[u(0),u(1),…,u(Nc​−1)]T。

    应用控制输入：

        将 u(0)u(0) 应用于系统。

    更新状态：

        在k=1时刻，测量新的状态 x(1)x(1)，并重复上述过程。

总结

    控制输入序列的假设：

        在优化求解之前，假设一个初始的控制输入序列。

    预测未来状态：

        基于假设的控制输入序列，预测未来 NpNp​ 步的状态。

    优化问题求解：

        通过优化求解，找到使预测状态与参考轨迹误差最小化的最优控制输入序列。

    优化求解的结果：

        优化求解出来的东西是最优的控制输入序列 U=[u(0),u(1),…,u(Nc−1)]TU=[u(0),u(1),…,u(Nc​−1)]T。

    是否需要重新预测：

        不需要重新预测，优化求解出来的控制输入序列可以直接用于控制。

    滚动优化：

        在每个时间步k，重新进行预测和优化，得到新的控制输入序列，并应用于系统。

这种方法确保了MPC控制器的实时性和鲁棒性，能够有效应对AUV在复杂海洋环境中的动态变化
*/

