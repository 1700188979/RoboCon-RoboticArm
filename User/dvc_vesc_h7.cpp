/**
 * @file dvc_vesc_h7.cpp
 * @brief vesc电机配置与操作 (FDCAN 版本)
 *
 */

#include "dvc_vesc_h7.h"

/**
 * @brief 初始化
 *
 * @param hfdcan FDCAN 句柄
 * @param __Motor_ID 电机ID
 *
 * 说明：这里只做管理对象绑定与 motor id 赋值，FDCAN 的回调注册请在上层做（参见下方示例）。
 */
void Class_Vesc::Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __Motor_ID)
{
    if (hfdcan == nullptr)
        return;

    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN_Manage_Object = &FDCAN1_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN_Manage_Object = &FDCAN2_Manage_Object;
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        FDCAN_Manage_Object = &FDCAN3_Manage_Object;
    }
    Motor_ID = __Motor_ID;
}

/**
 * @brief 定时器回调（用于发送目标速度）
 */
void Class_Vesc::TIM_Send_PeriodElapsedCallback()
{
    Send_Target_Vel_cmd();
}

/**
 * @brief 设置目标速度
 *
 * @param Input_Vel 目标速度 (RPM)
 */
void Class_Vesc::Set_Target_Vel(float Input_Vel)
{
    Target_Vel = Input_Vel;
}

/**
 * @brief 发送目标速度命令
 *
 * ID 构造方式保持原来逻辑：高字节为 packet id，低字节为 motor id
 */
void Class_Vesc::Send_Target_Vel_cmd()
{
    // 检查管理对象和 FDCAN 句柄
    if (FDCAN_Manage_Object == nullptr || FDCAN_Manage_Object->FDCAN_Handler == nullptr)
        return;

    // 构造扩展 ID：高字节为 packet id，低字节为 motor id
    uint32_t id = (CAN_PACKET_SET_RPM << 8) | Motor_ID;

    // 数据数组，保持小端顺序
    uint8_t data[4];
    int temp_vel = int(Target_Vel);
    Math_Endian_Reverse_32(&temp_vel);
    memcpy(data, &temp_vel, 4);

    // FDCAN 发送扩展帧
    FDCAN_Send_Data(FDCAN_Manage_Object->FDCAN_Handler, id, data, 4, FDCAN_EXTENDED_ID, FDCAN_DATA_FRAME);
}


// CAN 回调（直接传数组指针）
void Class_Vesc::FDCAN_RxCpltCallback(uint8_t *Rx_Data)
{
    if (Rx_Data == nullptr)
        return;

    Flag++;

    // 将接收到的数据指向管理对象，保持和原来一致性
    if (FDCAN_Manage_Object != nullptr)
    {
        memcpy(FDCAN_Manage_Object->Rx_Buffer.Data, Rx_Data, 8);
    }

    Data_Process();
}

/**
 * @brief 数据解析（把 8 字节 payload 解析成 rpm / current / duty）
 *
 * 注意：低字节在 temp[0]，保持 little-endian 拼接
 */
void Class_Vesc::Data_Process()
{
    if (FDCAN_Manage_Object == nullptr)
        return;

    uint8_t *temp = FDCAN_Manage_Object->Rx_Buffer.Data;
    if (temp == nullptr)
        return;

    // 解析 rpm (4 bytes, little-endian -> int32)
    rpm = static_cast<int32_t>((static_cast<int32_t>(temp[0]) << 24) |
                               (static_cast<int32_t>(temp[1]) << 16) |
                               (static_cast<int32_t>(temp[2]) << 8) |
                               (static_cast<int32_t>(temp[3])));

    // 解析电流 (bytes 4-5)，除以 10
    Toal_Current = static_cast<int16_t>(((static_cast<int16_t>(temp[5]) << 8) |
                                         (static_cast<int16_t>(temp[4]))) / 10);

    // 解析占空比 (bytes 6-7)，除以 1000
    Duty_Cycle = static_cast<int16_t>(((static_cast<int16_t>(temp[7]) << 8) |
                                      (static_cast<int16_t>(temp[6]))) / 1000);
}
