/**
 * file: s_curve_realtime.c
 * description: S 曲线轨迹规划（实时计算，无数组存储）
 * 适用平台: 嵌入式单片机
 *
 * 使用说明:
 *   1. 定义 SCurveParams 结构体变量
 *   2. 调用 SCurve_Init() 传入起始角度、结束角度、最大速度、最大加速度、总时间
 *   3. 在定时器中断或主循环中，根据当前时间 t 调用 SCurve_GetPosition(t) 获取角度
 *   4. 可同时获取速度、加速度
 */

#include <math.h>   // fabs, sqrt

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_ITER 20         // 迭代最大次数
#define EPS 1e-6f

/* 存储规划参数的结构体（无动态内存） */
typedef struct {
    float theta_start;      // 起始角度 (度)
    float delta;            // 总位移差 (度)
    float Tf;               // 修正后的总时间 (s)
    float V;                // 归一化峰值速度 (1/s)
    float A;                // 归一化峰值加速度 (1/s²)
    float J;                // 归一化加加速度 (1/s³)
    float T[7];             // 七段时间 (s)
} SCurveParams;

/* 内部函数声明 */
static void SCurvePara(float Tf_in, float v_in, float a_in,
                       float *Tf_out, float *V_out, float *A_out, float *J_out,
                       float T[7]);
static void SCurveScalingDerivatives(float t, float V, float A, float J, float T[7], float Tf,
                                     float *s, float *v, float *a);

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
 * @brief 计算归一化位移 s(t)、速度 v(t)、加速度 a(t)（解析公式）
 */
static void SCurveScalingDerivatives(float t, float V, float A, float J, float T[7], float Tf,
                                     float *s, float *v, float *a)
{
    float T1 = T[0], T2 = T[1], T3 = T[2], T4 = T[3];
    float T5 = T[4], T6 = T[5], T7 = T[6];
    float dt, t_temp;

    // 默认值（t 超出范围时 clamp）
    *s = 0.0f; *v = 0.0f; *a = 0.0f;

    if (t < 0.0f) {
        *s = 0.0f; *v = 0.0f; *a = 0.0f;
        return;
    }
    if (t > Tf) {
        *s = 1.0f; *v = 0.0f; *a = 0.0f;
        return;
    }

    // 阶段1：加加速段 (0 ~ T1)
    if (t <= T1) {
        *s = (1.0f/6.0f) * J * t * t * t;
        *v = 0.5f * J * t * t;
        *a = J * t;
    }
    // 阶段2：匀加速段 (T1 ~ T1+T2)
    else if (t <= T1 + T2) {
        dt = t - T1;
        *s = 0.5f * A * dt * dt
             + (A * A / (2.0f * J)) * dt
             + (A * A * A) / (6.0f * J * J);
        *v = A * dt + (A * A) / (2.0f * J);
        *a = A;
    }
    // 阶段3：减加速段 (T1+T2 ~ T1+T2+T3)
    else if (t <= T1 + T2 + T3) {
        dt = t - T1 - T2;
        *s = -1.0f/6.0f * J * dt * dt * dt
             + 0.5f * A * dt * dt
             + (A * T2 + A * A / (2.0f * J)) * dt
             + 0.5f * A * T2 * T2
             + (A * A / (2.0f * J)) * T2
             + (A * A * A) / (6.0f * J * J);
        *v = -0.5f * J * dt * dt + A * dt + (A * T2 + A * A / (2.0f * J));
        *a = -J * dt + A;
    }
    // 阶段4：匀速段 (T1+T2+T3 ~ T1+T2+T3+T4)
    else if (t <= T1 + T2 + T3 + T4) {
        dt = t - T1 - T2 - T3;
        float s_const = (-1.0f/6.0f * J * T3 * T3 * T3)
                        + 0.5f * A * T3 * T3
                        + (A * T2 + A * A / (2.0f * J)) * T3
                        + 0.5f * A * T2 * T2
                        + (A * A / (2.0f * J)) * T2
                        + (A * A * A) / (6.0f * J * J);
        *s = V * dt + s_const;
        *v = V;
        *a = 0.0f;
    }
    // 阶段5：加减速段（对称于阶段3）
    else if (t <= T1 + T2 + T3 + T4 + T5) {
        t_temp = Tf - t;
        dt = t_temp - T1 - T2;   // 对应阶段3的 dt
        float s_off = -1.0f/6.0f * J * dt * dt * dt
                      + 0.5f * A * dt * dt
                      + (A * T2 + A * A / (2.0f * J)) * dt
                      + 0.5f * A * T2 * T2
                      + (A * A / (2.0f * J)) * T2
                      + (A * A * A) / (6.0f * J * J);
        *s = 1.0f - s_off;
        *v = 0.5f * J * dt * dt - A * dt - (A * T2 + A * A / (2.0f * J));
        *v = -(*v);   // 速度方向相反（减速）
        *a = J * dt - A;
    }
    // 阶段6：匀减速段（对称于阶段2）
    else if (t <= T1 + T2 + T3 + T4 + T5 + T6) {
        t_temp = Tf - t;
        dt = t_temp - T1;
        float s_off = 0.5f * A * dt * dt
                      + (A * A / (2.0f * J)) * dt
                      + (A * A * A) / (6.0f * J * J);
        *s = 1.0f - s_off;
        *v = A * dt + (A * A) / (2.0f * J);
        *v = -(*v);
        *a = -A;
    }
    // 阶段7：减减速段（对称于阶段1）
    else {
        t_temp = Tf - t;
        *s = (1.0f/6.0f) * J * t_temp * t_temp * t_temp;
        *s = 1.0f - *s;
        *v = 0.5f * J * t_temp * t_temp;
        *v = -(*v);
        *a = -J * t_temp;
    }

    // 边界保护
    if (*s < 0.0f) *s = 0.0f;
    if (*s > 1.0f) *s = 1.0f;
    // 速度、加速度理论上不会超界，但为防止浮点误差，可加 clamp
}

/**
 * @brief 初始化 S 曲线规划参数（不生成数组，只存储参数）
 * @param theta_start  起始角度 (度)
 * @param theta_end    结束角度 (度)
 * @param V_theta      期望最大速度 (度/秒)
 * @param A_theta      期望最大加速度 (度/秒²)
 * @param Tf           期望总时间 (秒)
 * @param params       输出参数结构体（调用者提供地址）
 */
void SCurve_Init(float theta_start, float theta_end,
                 float V_theta, float A_theta, float Tf,
                 SCurveParams *params)
{
    float delta = theta_end - theta_start;
    float v = fabsf(V_theta / delta);
    float a = fabsf(A_theta / delta);

    // 计算修正后的 S 曲线参数
    SCurvePara(Tf, v, a, &params->Tf, &params->V, &params->A, &params->J, params->T);

    params->theta_start = theta_start;
    params->delta = delta;
}

/**
 * @brief 实时获取当前时刻的角度（度）
 * @param t       当前时间 (s)
 * @param params  已初始化的参数结构体
 * @return 角度值 (度)
 */
float SCurve_GetPosition(float t, const SCurveParams *params)
{
    float s, v, a;
    SCurveScalingDerivatives(t, params->V, params->A, params->J,
                             (float*)params->T, params->Tf, &s, &v, &a);
    return params->theta_start + s * params->delta;
}

/**
 * @brief 实时获取当前时刻的速度（度/秒）
 * @param t       当前时间 (s)
 * @param params  已初始化的参数结构体
 * @return 速度值 (度/秒)
 */
float SCurve_GetVelocity(float t, const SCurveParams *params)
{
    float s, v, a;
    SCurveScalingDerivatives(t, params->V, params->A, params->J,
                             (float*)params->T, params->Tf, &s, &v, &a);
    return v * params->delta;
}

/**
 * @brief 实时获取当前时刻的加速度（度/秒²）
 * @param t       当前时间 (s)
 * @param params  已初始化的参数结构体
 * @return 加速度值 (度/秒²)
 */
float SCurve_GetAcceleration(float t, const SCurveParams *params)
{
    float s, v, a;
    SCurveScalingDerivatives(t, params->V, params->A, params->J,
                             (float*)params->T, params->Tf, &s, &v, &a);
    return a * params->delta;
}


SCurveParams motion;

void main(void) {
    // 初始化：从 0° 到 90°，最大速度 100°/s，最大加速度 500°/s²，总时间 1.2 秒
    SCurve_Init(0.0f, 90.0f, 100.0f, 500.0f, 1.2f, &motion);

    // 假设在定时器中断中，每 10ms 调用一次，当前时间为 t
    float t = 0.0f;
    while (t <= motion.Tf) {
        float pos = SCurve_GetPosition(t, &motion);
        float vel = SCurve_GetVelocity(t, &motion);
        float acc = SCurve_GetAcceleration(t, &motion);
        // 发送给电机或记录
        // ...
        t += 0.001f;   // 10ms 步长
        printf("%f,%f,%f\n",pos,vel,acc);
    }
}