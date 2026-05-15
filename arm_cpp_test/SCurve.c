
/**
 * 使用说明:
 *   1. 定义 N 为离散点数（例如 200）
 *   2. 调用 SCurve_Init() 传入起始角度、结束角度、最大速度、最大加速度、总时间
 *   3. 调用 SCurve_Generate() 填充输出数组 theta[N]（角度序列）
 *
 * 可选: 获取归一化位移、速度、加速度数组
 */

#include <math.h>   // fabs, sqrt

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 配置宏 */
#define N 200               // 轨迹离散点数（可根据内存调整）
#define MAX_ITER 20         // SCurvePara 最大迭代次数（嵌入式一般很快收敛）
#define EPS 1e-6f

/* 静态工作缓冲区（避免堆分配） */
static float s_curve_s[N];      // 归一化位移
static float s_curve_sd[N];     // 归一化速度（一阶导）
static float s_curve_sdd[N];    // 归一化加速度（二阶导）
static float s_curve_t[N];      // 时间序列

/* 内部函数声明 */
static void SCurvePara(float Tf_in, float v_in, float a_in,
                       float *Tf_out, float *V_out, float *A_out, float *J_out,
                       float T[7]);
static float SCurveScaling(float t, float V, float A, float J, float T[7], float Tf);

/**
 * @brief 计算 S 曲线参数（迭代修正）
 * @param Tf_in    期望总时间 (s)
 * @param v_in     归一化期望速度 (1/s)
 * @param a_in     归一化期望加速度 (1/s²)
 * @param Tf_out   输出修正后总时间
 * @param V_out    输出实际归一化速度
 * @param A_out    输出实际归一化加速度
 * @param J_out    输出加加速度 (1/s³)
 * @param T        输出七段时间数组 (长度7)
 */
static void SCurvePara(float Tf_in, float v_in, float a_in,
                       float *Tf_out, float *V_out, float *A_out, float *J_out,
                       float T[7])
{
    float Tf = Tf_in;
    float v = v_in;
    float a = a_in;
    float J;
    int iter;
    float t1, t2, t4;

    for (iter = 0; iter < MAX_ITER; iter++) {
        float denom = Tf * v * a - v * v - a;
        if (fabsf(denom) < 1e-12f) denom = 1e-12f;
        J = (a * a * v) / denom;

        t1 = a / J;
        t2 = v / a - a / J;
        t4 = Tf - 2.0f * a / J - 2.0f * v / a;

        if (t2 < -EPS) {
            a = sqrtf(v * J);
        } else if (t4 < -EPS) {
            v = Tf * a / 2.0f - a * a / J;
        } else if (J < -EPS) {
            Tf = (v * v + a) / (v * a) + 0.1f;
        } else {
            break;
        }
    }

    *Tf_out = Tf;
    *V_out  = v;
    *A_out  = a;
    *J_out  = J;

    T[0] = t1;   // T1
    T[1] = t2;   // T2
    T[2] = t1;   // T3
    T[3] = t4;   // T4
    T[4] = t1;   // T5
    T[5] = t2;   // T6
    T[6] = t1;   // T7
}

/**
 * @brief 计算归一化位移 s(t)
 */
static float SCurveScaling(float t, float V, float A, float J, float T[7], float Tf)
{
    float s = 0.0f;
    float dt, t_temp;
    float T1 = T[0], T2 = T[1], T3 = T[2], T4 = T[3];
    float T5 = T[4], T6 = T[5];

    if (t >= 0.0f && t <= T1) {
        s = (1.0f/6.0f) * J * t * t * t;
    }
    else if (t > T1 && t <= T1 + T2) {
        dt = t - T1;
        s = 0.5f * A * dt * dt
            + (A * A / (2.0f * J)) * dt
            + (A * A * A) / (6.0f * J * J);
    }
    else if (t > T1 + T2 && t <= T1 + T2 + T3) {
        dt = t - T1 - T2;
        s = -1.0f/6.0f * J * dt * dt * dt
            + 0.5f * A * dt * dt
            + (A * T2 + A * A / (2.0f * J)) * dt
            + 0.5f * A * T2 * T2
            + (A * A / (2.0f * J)) * T2
            + (A * A * A) / (6.0f * J * J);
    }
    else if (t > T1 + T2 + T3 && t <= T1 + T2 + T3 + T4) {
        dt = t - T1 - T2 - T3;
        s = V * dt
            + (-1.0f/6.0f * J * T3 * T3 * T3)
            + 0.5f * A * T3 * T3
            + (A * T2 + A * A / (2.0f * J)) * T3
            + 0.5f * A * T2 * T2
            + (A * A / (2.0f * J)) * T2
            + (A * A * A) / (6.0f * J * J);
    }
    else if (t > T1 + T2 + T3 + T4 && t <= T1 + T2 + T3 + T4 + T5) {
        t_temp = Tf - t;
        dt = t_temp - T1 - T2;
        s = -1.0f/6.0f * J * dt * dt * dt
            + 0.5f * A * dt * dt
            + (A * T2 + A * A / (2.0f * J)) * dt
            + 0.5f * A * T2 * T2
            + (A * A / (2.0f * J)) * T2
            + (A * A * A) / (6.0f * J * J);
        s = 1.0f - s;
    }
    else if (t > T1 + T2 + T3 + T4 + T5 && t <= T1 + T2 + T3 + T4 + T5 + T6) {
        t_temp = Tf - t;
        dt = t_temp - T1;
        s = 0.5f * A * dt * dt
            + (A * A / (2.0f * J)) * dt
            + (A * A * A) / (6.0f * J * J);
        s = 1.0f - s;
    }
    else if (t > T1 + T2 + T3 + T4 + T5 + T6 && t <= Tf) {
        t_temp = Tf - t;
        s = (1.0f/6.0f) * J * t_temp * t_temp * t_temp;
        s = 1.0f - s;
    }

    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    return s;
}

/**
 * @brief 生成 S 曲线轨迹（角度序列）
 * @param ThetaStart  起始角度 (度)
 * @param ThetaEnd    结束角度 (度)
 * @param VTheta      期望最大速度 (度/秒)
 * @param ATheta      期望最大加速度 (度/秒²)
 * @param Tf          期望总时间 (秒)
 * @param theta_out   输出数组，长度为 N，调用者需保证有足够空间
 */
void SCurve_Generate(float ThetaStart, float ThetaEnd,
                     float VTheta, float ATheta, float Tf,
                     float *theta_out)
{
    float delta = ThetaEnd - ThetaStart;
    float v = fabsf(VTheta / delta);
    float a = fabsf(ATheta / delta);
    float Tf_adj, V_adj, A_adj, J_adj;
    float Tseg[7];

    // 计算修正后的 S 曲线参数
    SCurvePara(Tf, v, a, &Tf_adj, &V_adj, &A_adj, &J_adj, Tseg);

    // 生成时间序列 (均匀采样)
    float dt = Tf_adj / (N - 1);
    for (int i = 0; i < N; i++) {
        s_curve_t[i] = i * dt;
        s_curve_s[i] = SCurveScaling(s_curve_t[i], V_adj, A_adj, J_adj, Tseg, Tf_adj);
        theta_out[i] = ThetaStart + s_curve_s[i] * delta;
    }

    // 可选：计算速度、加速度（供需要时使用）
    for (int i = 0; i < N - 1; i++) {
        s_curve_sd[i] = (s_curve_s[i+1] - s_curve_s[i]) / dt;
    }
    s_curve_sd[N-1] = s_curve_sd[N-2];

    for (int i = 0; i < N - 2; i++) {
        s_curve_sdd[i] = (s_curve_sd[i+1] - s_curve_sd[i]) / dt;
    }
    s_curve_sdd[N-2] = s_curve_sdd[N-3];
    s_curve_sdd[N-1] = s_curve_sdd[N-3];
}

/* 如果需要同时获取速度、加速度，可添加以下接口（可选） */
void SCurve_GenerateFull(float ThetaStart, float ThetaEnd,
                         float VTheta, float ATheta, float Tf,
                         float *theta_out, float *vel_out, float *acc_out)
{
    SCurve_Generate(ThetaStart, ThetaEnd, VTheta, ATheta, Tf, theta_out);
    for (int i = 0; i < N; i++) {
        vel_out[i] = s_curve_sd[i] * (ThetaEnd - ThetaStart);
        acc_out[i] = s_curve_sdd[i] * (ThetaEnd - ThetaStart);
    }
}
#include "stdio.h"
float angles[N];
float vel[N];
float acc[N];
int i=0;
void main(void) {
    // 从 0° 运动到 90°，最大速度 100°/s，最大加速度 500°/s²，总时间 1.2s
    SCurve_GenerateFull(0.0f, 90.0f, 100.0f, 500.0f, 1.2f, angles, vel, acc);
    for(i=0;i<N;i++){
        printf("%f  %f  %f\n", angles[i], vel[i], acc[i]);
    }
    // 之后可把 angles 发送给伺服驱动器，或用于位置插补
}