/**
 * @file dvc_motor_dji_h7.h
 * @brief 大疆电机配置与操作
 * @brief  官方6020驱动更新, 适配电压控制与电流控制
 *
 */

#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_fdcan_h7.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆状态
 *
 */
enum Enum_Motor_DJI_Status
{
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
};

/**
 * @brief 大疆电机的ID枚举类型
 *
 */
enum Enum_Motor_DJI_ID
{
    Motor_DJI_ID_0x201 = 1,
    Motor_DJI_ID_0x202,
    Motor_DJI_ID_0x203,
    Motor_DJI_ID_0x204,
    Motor_DJI_ID_0x205,
    Motor_DJI_ID_0x206,
    Motor_DJI_ID_0x207,
    Motor_DJI_ID_0x208,
    Motor_DJI_ID_0x209,
    Motor_DJI_ID_0x20A,
    Motor_DJI_ID_0x20B,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum Enum_Motor_DJI_Control_Method
{
    Motor_DJI_Control_Method_VOLTAGE = 0,
    Motor_DJI_Control_Method_CURRENT,
    Motor_DJI_Control_Method_TORQUE,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
};

/**
 * @brief 大疆电机驱动版本, 影响GM6020电机驱动方式
 *
 */
enum Enum_Motor_DJI_GM6020_Driver_Version
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_2023,
};

/**
 * @brief 大疆电机源数据
 *
 */
struct Struct_Motor_DJI_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理的数据
 *
 */
struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/**
 * @brief Reusable, GM6020无刷电机, 单片机控制输出电压, 新驱动支持电流
 *
 */
class Class_Motor_DJI_GM6020
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;
    // PID电流环控制
    Class_PID PID_Current;

    void Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE, int32_t __Encoder_Offset = 0, Enum_Motor_DJI_GM6020_Driver_Version __Drive_Version = Motor_DJI_GM6020_Driver_Version_2023, float __Voltage_Max = 24.0f, float __Current_Max = 3.0f);

    inline float Get_Voltage_Max();

    inline float Get_Current_Max();

    inline float Get_Theoretical_Output_Voltage_Max();

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_DJI_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Current();

    inline float Get_Now_Temperature();

    inline Enum_Motor_DJI_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Target_Voltage();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline float Get_Feedforward_Voltage();

    inline void Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Target_Voltage(float __Voltage);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    inline void Set_Feedforward_Voltage(float __Feedforward_Voltage);

    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的 FDCAN 管理对象
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;
    // 收数据绑定的FDCAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 电机驱动方式
    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
    // 最大电压
    float Voltage_Max;
    // 最大电流
    float Current_Max;

    // 常量

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电压到输出的转化系数
    float Voltage_To_Out = 25000.0f / 24.0f;
    // 电流到输出的转化系数
    float Current_To_Out = 16384.0f / 3.0f;
    // 理论最大输出电压
    float Theoretical_Output_Voltage_Max = 24.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 3.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流, A
    float Target_Current = 0.0f;
    // 目标的电压, V
    float Target_Voltage = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    // 前馈的电压, V
    float Feedforward_Voltage = 0.0f;

    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};

/**
 * @brief Reusable, C610无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_C610
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, float __Gearbox_Rate = 36.0f, float __Current_Max = 10.0f);

    inline float Get_Current_Max();

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_DJI_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Current();

    inline uint8_t Get_Now_Temperature();

    inline Enum_Motor_DJI_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline void Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

    void Set_Angle_Clear();

protected:
    // 初始化相关常量

    // 绑定的FDCAN管理对象
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;
    // 收数据绑定的FDCAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;
    // 最大电流
    float Current_Max;

    // 常量

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电流到输出的转化系数
    float Current_To_Out = 10000.0f / 10.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 10.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流, A
    float Target_Current = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    //清除标志位
    uint8_t Angle_Clear_Flag = 0;
    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};

/**
 * @brief Reusable, C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_C620
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, float __Gearbox_Rate = 19.0f, float __Current_Max = 20.0f);

    inline float Get_Current_Max();

    inline float Get_Theoretical_Output_Current_Max();

    inline Enum_Motor_DJI_Status Get_Status();

    inline float Get_Now_Angle();

    inline float Get_Now_Omega();

    inline float Get_Now_Current();

    inline float Get_Now_Temperature();

    inline Enum_Motor_DJI_Control_Method Get_Control_Method();

    inline float Get_Target_Angle();

    inline float Get_Target_Omega();

    inline float Get_Target_Current();

    inline float Get_Feedforward_Omega();

    inline float Get_Feedforward_Current();

    inline void Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method);

    inline void Set_Target_Angle(float __Target_Angle);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Target_Current(float __Target_Current);

    inline void Set_Feedforward_Omega(float __Feedforward_Omega);

    inline void Set_Feedforward_Current(float __Feedforward_Current);

    void Set_Angle_Clear();

    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();


protected:
    // 初始化相关变量

    // 绑定的FDCAN管理对象
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object;
    // 收数据绑定的FDCAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;
    // 最大电流
    float Current_Max;

    // 常量

    // 一圈编码器刻度
    uint16_t Encoder_Num_Per_Round = 8192;

    // 电流到输出的转化系数
    float Current_To_Out = 16384.0f / 20.0f;
    // 理论最大输出电流
    float Theoretical_Output_Current_Max = 20.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的电流
    float Target_Current = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的电流, A
    float Feedforward_Current = 0.0f;
    //清除标志位
    uint8_t Angle_Clear_Flag = 0;
    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline float Class_Motor_DJI_GM6020::Get_Voltage_Max()
{
    return (Voltage_Max);
}

inline float Class_Motor_DJI_GM6020::Get_Current_Max()
{
    return (Current_Max);
}

inline float Class_Motor_DJI_GM6020::Get_Theoretical_Output_Voltage_Max()
{
    return (Theoretical_Output_Voltage_Max);
}

inline float Class_Motor_DJI_GM6020::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

inline Enum_Motor_DJI_Status Class_Motor_DJI_GM6020::Get_Status()
{
    return (Motor_DJI_Status);
}

inline float Class_Motor_DJI_GM6020::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

inline float Class_Motor_DJI_GM6020::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

inline float Class_Motor_DJI_GM6020::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

inline float Class_Motor_DJI_GM6020::Get_Now_Temperature()
{
    return (Rx_Data.Now_Temperature);
}

inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_GM6020::Get_Control_Method()
{
    return (Motor_DJI_Control_Method);
}

inline float Class_Motor_DJI_GM6020::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Motor_DJI_GM6020::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_Motor_DJI_GM6020::Get_Target_Current()
{
    return (Target_Current);
}

inline float Class_Motor_DJI_GM6020::Get_Target_Voltage()
{
    return (Target_Voltage);
}

inline float Class_Motor_DJI_GM6020::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

inline float Class_Motor_DJI_GM6020::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

inline float Class_Motor_DJI_GM6020::Get_Feedforward_Voltage()
{
    return (Feedforward_Voltage);
}

inline void Class_Motor_DJI_GM6020::Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

inline void Class_Motor_DJI_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DJI_GM6020::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Motor_DJI_GM6020::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

inline void Class_Motor_DJI_GM6020::Set_Target_Voltage(float __Voltage)
{
    Target_Voltage = __Voltage;
}

inline void Class_Motor_DJI_GM6020::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

inline void Class_Motor_DJI_GM6020::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

inline void Class_Motor_DJI_GM6020::Set_Feedforward_Voltage(float __Feedforward_Voltage)
{
    Feedforward_Voltage = __Feedforward_Voltage;
}

inline float Class_Motor_DJI_C610::Get_Current_Max()
{
    return (Current_Max);
}

inline float Class_Motor_DJI_C610::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

inline Enum_Motor_DJI_Status Class_Motor_DJI_C610::Get_Status()
{
    return (Motor_DJI_Status);
}

inline float Class_Motor_DJI_C610::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

inline float Class_Motor_DJI_C610::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

inline float Class_Motor_DJI_C610::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

inline uint8_t Class_Motor_DJI_C610::Get_Now_Temperature()
{
    return (Rx_Data.Now_Temperature);
}

inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_C610::Get_Control_Method()
{
    return (Motor_DJI_Control_Method);
}

inline float Class_Motor_DJI_C610::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Motor_DJI_C610::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_Motor_DJI_C610::Get_Target_Current()
{
    return (Target_Current);
}

inline float Class_Motor_DJI_C610::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

inline float Class_Motor_DJI_C610::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

inline void Class_Motor_DJI_C610::Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

inline void Class_Motor_DJI_C610::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DJI_C610::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Motor_DJI_C610::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

inline void Class_Motor_DJI_C610::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

inline void Class_Motor_DJI_C610::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

inline float Class_Motor_DJI_C620::Get_Current_Max()
{
    return (Current_Max);
}

inline float Class_Motor_DJI_C620::Get_Theoretical_Output_Current_Max()
{
    return (Theoretical_Output_Current_Max);
}

inline Enum_Motor_DJI_Status Class_Motor_DJI_C620::Get_Status()
{
    return (Motor_DJI_Status);
}

inline float Class_Motor_DJI_C620::Get_Now_Angle()
{
    return (Rx_Data.Now_Angle);
}

inline float Class_Motor_DJI_C620::Get_Now_Omega()
{
    return (Rx_Data.Now_Omega);
}

inline float Class_Motor_DJI_C620::Get_Now_Current()
{
    return (Rx_Data.Now_Current);
}

inline float Class_Motor_DJI_C620::Get_Now_Temperature()
{
    return (Rx_Data.Now_Temperature);
}

inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_C620::Get_Control_Method()
{
    return (Motor_DJI_Control_Method);
}

inline float Class_Motor_DJI_C620::Get_Target_Angle()
{
    return (Target_Angle);
}

inline float Class_Motor_DJI_C620::Get_Target_Omega()
{
    return (Target_Omega);
}

inline float Class_Motor_DJI_C620::Get_Target_Current()
{
    return (Target_Current);
}

inline float Class_Motor_DJI_C620::Get_Feedforward_Omega()
{
    return (Feedforward_Omega);
}

inline float Class_Motor_DJI_C620::Get_Feedforward_Current()
{
    return (Feedforward_Current);
}

inline void Class_Motor_DJI_C620::Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

inline void Class_Motor_DJI_C620::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

inline void Class_Motor_DJI_C620::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

inline void Class_Motor_DJI_C620::Set_Target_Current(float __Target_Current)
{
    Target_Current = __Target_Current;
}

inline void Class_Motor_DJI_C620::Set_Feedforward_Omega(float __Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

inline void Class_Motor_DJI_C620::Set_Feedforward_Current(float __Feedforward_Current)
{
    Feedforward_Current = __Feedforward_Current;
}

#endif
