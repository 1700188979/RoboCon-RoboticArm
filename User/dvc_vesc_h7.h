/**
 * @file dvc_vesc_h7.h
 * @brief vesc电机配置与操作 (FDCAN 版本)
 *
 */

#ifndef DVC_VESC_H
#define DVC_VESC_H

/* Includes ------------------------------------------------------------------*/

#include "drv_fdcan_h7.h"
#include "drv_math.h"
#include <cstring>
#include <cstdint>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_FILL_RX_BUFFER,
    CAN_PACKET_FILL_RX_BUFFER_LONG,
    CAN_PACKET_PROCESS_RX_BUFFER,
    CAN_PACKET_PROCESS_SHORT_BUFFER,
    CAN_PACKET_STATUS,
    CAN_PACKET_SET_CURRENT_REL,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_STATUS_2,
    CAN_PACKET_STATUS_3,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_PING,
    CAN_PACKET_PONG,
    CAN_PACKET_DETECT_APPLY_ALL_FOC,
    CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
    CAN_PACKET_CONF_CURRENT_LIMITS,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
    CAN_PACKET_CONF_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
    CAN_PACKET_CONF_FOC_ERPMS,
    CAN_PACKET_CONF_STORE_FOC_ERPMS,
    CAN_PACKET_STATUS_5,
    CAN_PACKET_POLL_TS5700N8501_STATUS,
    CAN_PACKET_CONF_BATTERY_CUT,
    CAN_PACKET_CONF_STORE_BATTERY_CUT,
    CAN_PACKET_SHUTDOWN,
    CAN_PACKET_IO_BOARD_ADC_1_TO_4,
    CAN_PACKET_IO_BOARD_ADC_5_TO_8,
    CAN_PACKET_IO_BOARD_ADC_9_TO_12,
    CAN_PACKET_IO_BOARD_DIGITAL_IN,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
    CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
    CAN_PACKET_BMS_V_TOT,
    CAN_PACKET_BMS_I,
    CAN_PACKET_BMS_AH_WH,
    CAN_PACKET_BMS_V_CELL,
    CAN_PACKET_BMS_BAL,
    CAN_PACKET_BMS_TEMPS,
    CAN_PACKET_BMS_HUM,
    CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
} CAN_PACKET_ID;

struct Vesc_Data_t
{
    // 可扩展：把你想缓存的更多状态放这里
};

/**
 * @brief VESC 驱动类（FDCAN 版本）
 *
 * 说明：
 *  - Init 接受 FDCAN_HandleTypeDef* 和 motor id（0..255）
 *  - 将回调注册到 drv_fdcan 时，需要把收到的 Struct_FDCAN_Rx_Buffer*
 *    交给实例的 FDCAN_RxCpltCallback 处理（文件末尾示例说明）。
 */
class Class_Vesc
{
public:
    // 初始化，传入 FDCAN 句柄与电机 ID（0..255）
    void Init(FDCAN_HandleTypeDef *hfdcan, uint8_t __Motor_ID);

    // 设置目标转速 (RPM)
    void Set_Target_Vel(float Input_Vel);

    // 读取当前速度（返回浮点，内部 rpm 为 int32）
    inline float Get_Now_Vel();

    // 定时回调（定时器触发用于发送目标速度）
    void TIM_Send_PeriodElapsedCallback();

    // FDCAN 接收回调（与 drv_fdcan 的 FDCAN_Call_Back 匹配）
    void FDCAN_RxCpltCallback(uint8_t *Rx_Data);

protected:
    // 绑定的 FDCAN 管理对象（drv_fdcan 提供的结构体）
    Struct_FDCAN_Manage_Object *FDCAN_Manage_Object = nullptr;

    // 电机ID
    uint8_t Motor_ID = 0;

    // 目标转速
    float Target_Vel = 0.0f;

    // 接收标志位（可用于外部判断是否有新数据）
    uint32_t Flag = 0;

    // 接收数据缓存
    int32_t rpm = 0;
    int16_t Toal_Current = 0;
    int16_t Duty_Cycle = 0;

    // 数据处理函数（把 Rx_Buffer 的原始 bytes 解析为成员）
    void Data_Process();

    // 发送目标转速命令（内部使用）
    void Send_Target_Vel_cmd();
};

inline float Class_Vesc::Get_Now_Vel()
{
    return static_cast<float>(rpm);
}

#endif // DVC_VESC_H
