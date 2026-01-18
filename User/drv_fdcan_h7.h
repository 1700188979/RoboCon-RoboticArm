/**
 * @file drv_fdcan_h7.h
 * @brief FDCAN通信初始化与配置流程（适配 STM32H7）
 */

#ifndef DRV_FDCAN_H
#define DRV_FDCAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include <cstring>
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief FDCAN 接收的信息结构体
 */
struct Struct_FDCAN_Rx_Buffer
{
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief FDCAN 通信接收回调函数数据类型
 */
typedef void (*FDCAN_Call_Back)(Struct_FDCAN_Rx_Buffer *);

/**
 * @brief FDCAN 通信处理结构体
 */
struct Struct_FDCAN_Manage_Object
{
    FDCAN_HandleTypeDef *FDCAN_Handler;
    Struct_FDCAN_Rx_Buffer Rx_Buffer;
    FDCAN_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern Struct_FDCAN_Manage_Object FDCAN1_Manage_Object;
extern Struct_FDCAN_Manage_Object FDCAN2_Manage_Object;
extern Struct_FDCAN_Manage_Object FDCAN3_Manage_Object;

/* 发送缓冲（保留原 ID 命名便于与上层协议对应） */
extern uint8_t FDCAN1_0x1fe_Tx_Data[8];
extern uint8_t FDCAN1_0x1ff_Tx_Data[8];
extern uint8_t FDCAN1_0x200_Tx_Data[8];
extern uint8_t FDCAN1_0x2fe_Tx_Data[8];
extern uint8_t FDCAN1_0x2ff_Tx_Data[8];
extern uint8_t FDCAN1_0x3fe_Tx_Data[8];
extern uint8_t FDCAN1_0x4fe_Tx_Data[8];

extern uint8_t FDCAN2_0x1fe_Tx_Data[8];
extern uint8_t FDCAN2_0x1ff_Tx_Data[8];
extern uint8_t FDCAN2_0x200_Tx_Data[8];
extern uint8_t FDCAN2_0x2fe_Tx_Data[8];
extern uint8_t FDCAN2_0x2ff_Tx_Data[8];
extern uint8_t FDCAN2_0x3fe_Tx_Data[8];
extern uint8_t FDCAN2_0x4fe_Tx_Data[8];

extern uint8_t FDCAN3_0x1fe_Tx_Data[8];
extern uint8_t FDCAN3_0x1ff_Tx_Data[8];
extern uint8_t FDCAN3_0x200_Tx_Data[8];
extern uint8_t FDCAN3_0x2fe_Tx_Data[8];
extern uint8_t FDCAN3_0x2ff_Tx_Data[8];
extern uint8_t FDCAN3_0x3fe_Tx_Data[8];
extern uint8_t FDCAN3_0x4fe_Tx_Data[8];

/* Exported macros (用于构造旧版移植时的位域表达) -------------------------------*/
#define FDCAN_FILTER(x) ((x) << 3)
#define FDCAN_FIFO_0 (0 << 2)
#define FDCAN_FIFO_1 (1 << 2)
#define FDCAN_STDID 0x00000000U
#define FDCAN_EXTID 0x40000000U
#define FDCAN_DATA_TYPE 0x00000000U
#define FDCAN_REMOTE_TYPE 0x20000000U

/* Exported function declarations ------------------- --------------------------*/

/**
 * @brief 初始化 FDCAN（代替原 CAN_Init）
 *
 * @param hfdcan FDCAN 句柄（CubeMX 默认名 hfdcan1/hfdcan2）
 * @param Callback_Function 接收回调（收到帧时会调用）
 */
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, FDCAN_Call_Back Callback_Function);

/**
 * @brief 发送数据帧
 *
 * @param hfdcan FDCAN 句柄
 * @param ID 标准ID（11bit）
 * @param Data 数据缓冲
 * @param Length 数据长度 (0..8)
 * @param Frame_type 使用 FDCAN_DATA_TYPE 或 FDCAN_REMOTE_TYPE
 * @return uint8_t 返回 HAL 状态（0 == HAL_OK）
 */
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *Data, uint16_t Length, uint32_t IDR_type,uint32_t Frame_type);

/**
 * @brief 1ms 定时器回调（如果你在原代码使用 TIM_1ms_CAN_PeriodElapsedCallback，可以替换为此函数）
 */
void TIM_1ms_FDCAN_PeriodElapsedCallback(void);

#endif /* DRV_FDCAN_H */
