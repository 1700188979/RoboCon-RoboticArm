/**
 * @file dvc_motor_dji_h7.cpp
 * @brief 大疆电机配置与操作（FDCAN 版）
 * @brief 官方6020驱动更新, 适配电压控制与电流控制
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji_h7.h"
#include "drv_fdcan_h7.h" // 已在 header 中包含，但为确保可见性再次包含

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配FDCAN发送缓冲区
 *
 * @param hfdcan FDCAN句柄
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Driver_Version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_ID, Enum_Motor_DJI_GM6020_Driver_Version __DJI_Motor_Driver_Version = Motor_DJI_GM6020_Driver_Version_2023)
{
    uint8_t *tmp_tx_data_ptr = nullptr;

    if (hfdcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(FDCAN1_0x200_Tx_Data[0]);
            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(FDCAN1_0x200_Tx_Data[2]);
            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(FDCAN1_0x200_Tx_Data[4]);
            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(FDCAN1_0x200_Tx_Data[6]);
            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1fe_Tx_Data[4]);
            }
            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1ff_Tx_Data[6]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x1fe_Tx_Data[6]);
            }
            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN1_0x2fe_Tx_Data[4]);
            }
            break;
        }
        default:
            break;
        }
    }
    else if (hfdcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(FDCAN2_0x200_Tx_Data[0]);
            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(FDCAN2_0x200_Tx_Data[2]);
            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(FDCAN2_0x200_Tx_Data[4]);
            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(FDCAN2_0x200_Tx_Data[6]);
            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1fe_Tx_Data[4]);
            }
            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1ff_Tx_Data[6]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x1fe_Tx_Data[6]);
            }
            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(FDCAN2_0x2fe_Tx_Data[4]);
            }
            break;
        }
        default:
            break;
        }
    }
    return (tmp_tx_data_ptr);
}


/**
 * @brief 电机初始化
 *
 * @param hfdcan 绑定的FDCAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Driver_Version 6020电机驱动方式, 默认旧版电压控制
 * @param __Voltage_Max 最大速度, 需根据不同负载测量后赋值
 * @param __Current_Max 最大电流
 */
void Class_Motor_DJI_GM6020::Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, int32_t __Encoder_Offset, Enum_Motor_DJI_GM6020_Driver_Version __Driver_Version, float __Voltage_Max, float __Current_Max)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
    Encoder_Offset = __Encoder_Offset;
    Driver_Version = __Driver_Version;
    Voltage_Max = __Voltage_Max;
    Current_Max = __Current_Max;
    Tx_Data = allocate_tx_data(hfdcan, __CAN_Rx_ID, __Driver_Version);
}

/**
 * @brief FDCAN通信接收回调函数（类内部处理函数）
 *
 * @param Rx_Data 接收的数据（本函数保留原签名，但内部使用管理对象）
 */
void Class_Motor_DJI_GM6020::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_GM6020::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        PID_Current.Set_Integral_Error(0.0f);
    }
    else
    {
        // 电机保持连接
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_DJI_GM6020::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
    {
        float tmp_value = Target_Voltage + Feedforward_Voltage;
        Math_Constrain(&tmp_value, -Voltage_Max, Voltage_Max);
        Out = tmp_value * Voltage_To_Out;
    }
    else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
    {
        float tmp_value = Target_Current + Feedforward_Current;
        Math_Constrain(&tmp_value, -Current_Max, Current_Max);
        Out = tmp_value * Current_To_Out;
    }

    Output();
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DJI_GM6020::Data_Process()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) FDCAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse, (void *)&tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        // 正方向转过了一圈
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        // 反方向转过了一圈
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder + Encoder_Offset;

    // 计算电机本身信息
    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS;
    Rx_Data.Now_Current = tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DJI_GM6020::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_VOLTAGE):
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            // nothing special
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            Target_Voltage = 0.0f;
            Target_Current = 0.0f;
        }
        break;
    }
    case (Motor_DJI_Control_Method_CURRENT):
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            PID_Current.Set_Target(Target_Current + Feedforward_Current);
            PID_Current.Set_Now(Rx_Data.Now_Current);
            PID_Current.TIM_Calculate_PeriodElapsedCallback();

            Target_Voltage = PID_Current.Get_Out();
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            // new-driver path (no-op here)
        }
        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            PID_Current.Set_Target(Target_Current + Feedforward_Current);
            PID_Current.Set_Now(Rx_Data.Now_Current);
            PID_Current.TIM_Calculate_PeriodElapsedCallback();

            Target_Voltage = PID_Current.Get_Out();
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();
        }
        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            PID_Current.Set_Target(Target_Current + Feedforward_Current);
            PID_Current.Set_Now(Rx_Data.Now_Current);
            PID_Current.TIM_Calculate_PeriodElapsedCallback();

            Target_Voltage = PID_Current.Get_Out();
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();
        }
        break;
    }
    default:
    {
        if (Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            Target_Voltage = 0.0f;
        }
        else if (Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            Target_Current = 0.0f;
        }
        break;
    }
    }
}

/**
 * @brief 电机数据输出到FDCAN发送缓冲区
 *
 */
void Class_Motor_DJI_GM6020::Output()
{
    int16_t tmp = (int16_t)Out;
    Tx_Data[0] = (uint8_t)(tmp >> 8);
    Tx_Data[1] = (uint8_t)(tmp & 0xFF);

}

/**
 * @brief C610 初始化
 *
 */
void Class_Motor_DJI_C610::Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Gearbox_Rate, float __Current_Max)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Current_Max = __Current_Max;
    Tx_Data = allocate_tx_data(hfdcan, __CAN_Rx_ID);
}

/**
 * @brief C610 接收回调（类内）
 */
void Class_Motor_DJI_C610::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;
    Data_Process();
}

/**
 * @brief C610 存活检测
 */
void Class_Motor_DJI_C610::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief C610 计算回调
 */
void Class_Motor_DJI_C610::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    float tmp_value = Target_Current + Feedforward_Current;
    Math_Constrain(&tmp_value, -Current_Max, Current_Max);
    Out = tmp_value * Current_To_Out;

    Output();

    Feedforward_Current = 0.0f;
    Feedforward_Omega = 0.0f;
}

/**
 * @brief C610 数据处理
 */
void Class_Motor_DJI_C610::Data_Process()
{
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    uint16_t tmp_clear_encoder = 0;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *)FDCAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse, (void *)&tmp_current);

    if (Angle_Clear_Flag == 1)
    {
        tmp_clear_encoder = tmp_encoder;
        Angle_Clear_Flag = 0;
    }

    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder - tmp_clear_encoder;

    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Current = tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief C610 PID 计算
 */
void Class_Motor_DJI_C610::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_CURRENT):
    {
        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();

        break;
    }
    default:
    {
        Target_Current = 0.0f;
        break;
    }
    }
}

/**
 * @brief C610 输出到发送缓冲
 */
void Class_Motor_DJI_C610::Output()
{
    int16_t tmp = (int16_t)Out;
    Tx_Data[0] = (uint8_t)(tmp >> 8);
    Tx_Data[1] = (uint8_t)(tmp & 0xFF);
}

/**
 * @brief C610 编码器清零
 */
void Class_Motor_DJI_C610::Set_Angle_Clear()
{
    Angle_Clear_Flag = 1;
}

/**
 * @brief C620 Init
 */
void Class_Motor_DJI_C620::Init(FDCAN_HandleTypeDef *hfdcan, Enum_Motor_DJI_ID __CAN_Rx_ID, Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method, float __Gearbox_Rate, float __Current_Max)
{
    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Current_Max = __Current_Max;
    Tx_Data = allocate_tx_data(hfdcan, __CAN_Rx_ID);
}

/**
 * @brief C620 接收回调
 */
void Class_Motor_DJI_C620::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    Flag += 1;
    Data_Process();
}

/**
 * @brief C620 存活检测
 */
void Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
        Motor_DJI_Control_Method = Motor_DJI_Control_Method_CURRENT;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Target_Current = 0.0f;
    }
    else
    {
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief C620 计算回调
 */
void Class_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    float tmp_value = Target_Current + Feedforward_Current;
    Math_Constrain(&tmp_value, -Current_Max, Current_Max);
    Out = tmp_value * Current_To_Out;

    Output();
}

/**
 * @brief C620 数据处理
 */
void Class_Motor_DJI_C620::Data_Process()
{
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    uint16_t tmp_clear_encoder = 0;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *)FDCAN_Manage_Object->Rx_Buffer.Data;

    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse, (void *)&tmp_current);

    if (Angle_Clear_Flag == 1)
    {
        tmp_clear_encoder = tmp_encoder;
        Angle_Clear_Flag = 0;
    }

    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > Encoder_Num_Per_Round / 2)
    {
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder - tmp_clear_encoder;

    Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / (float)Encoder_Num_Per_Round * 2.0f * PI / Gearbox_Rate;
    Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Current = tmp_current / Current_To_Out;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief C620 PID 计算
 */
void Class_Motor_DJI_C620::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_CURRENT):
    {
        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Current = PID_Omega.Get_Out();

        break;
    }
    default:
    {
        Target_Current = 0.0f;
        break;
    }
    }
}

/**
 * @brief C620 输出到发送缓冲
 */
void Class_Motor_DJI_C620::Output()
{
    int16_t tmp = (int16_t)Out;
    Tx_Data[0] = (uint8_t)(tmp >> 8);
    Tx_Data[1] = (uint8_t)(tmp & 0xFF);
}

/**
 * @brief C620 编码器清零
 */
void Class_Motor_DJI_C620::Set_Angle_Clear()
{
    Angle_Clear_Flag = 1;
}
