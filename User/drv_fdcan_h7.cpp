/**
 * @file drv_fdcan_h7.cpp
 * @brief FDCAN通信初始化与配置流程（适配 STM32H7）
 *
 * 由原 drv_can.cpp 移植，保留原逻辑与回调风格，替换为 HAL FDCAN API。
 */

#include "drv_fdcan_h7.h"

/* Private variables ---------------------------------------------------------*/

Struct_FDCAN_Manage_Object FDCAN1_Manage_Object = {0};
Struct_FDCAN_Manage_Object FDCAN2_Manage_Object = {0};
Struct_FDCAN_Manage_Object FDCAN3_Manage_Object = {0};

// 发送缓冲区（保留原 ID 名称）
uint8_t FDCAN1_0x1fe_Tx_Data[8];
uint8_t FDCAN1_0x1ff_Tx_Data[8];
uint8_t FDCAN1_0x200_Tx_Data[8];
uint8_t FDCAN1_0x2fe_Tx_Data[8];
uint8_t FDCAN1_0x2ff_Tx_Data[8];
uint8_t FDCAN1_0x3fe_Tx_Data[8];
uint8_t FDCAN1_0x4fe_Tx_Data[8];

uint8_t FDCAN2_0x1fe_Tx_Data[8];
uint8_t FDCAN2_0x1ff_Tx_Data[8];
uint8_t FDCAN2_0x200_Tx_Data[8];
uint8_t FDCAN2_0x2fe_Tx_Data[8];
uint8_t FDCAN2_0x2ff_Tx_Data[8];
uint8_t FDCAN2_0x3fe_Tx_Data[8];
uint8_t FDCAN2_0x4fe_Tx_Data[8];

uint8_t FDCAN3_0x1fe_Tx_Data[8];
uint8_t FDCAN3_0x1ff_Tx_Data[8];
uint8_t FDCAN3_0x200_Tx_Data[8];
uint8_t FDCAN3_0x2fe_Tx_Data[8];
uint8_t FDCAN3_0x2ff_Tx_Data[8];
uint8_t FDCAN3_0x3fe_Tx_Data[8];
uint8_t FDCAN3_0x4fe_Tx_Data[8];

/* Forward declarations */
static void fdcan_filter_mask_config(FDCAN_HandleTypeDef *hfdcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief 配置FDCAN的过滤器（简化映射，尽量保留原 can_filter_mask_config 的意图）
 *
 * 注意：FDCAN 过滤器配置与 bxCAN 不同。此函数以兼容姿态设置：
 *   - 如果 Object_Para 表示标准帧，则使用 FDCAN_STANDARD_ID；否则扩展帧。
 *   - FilterIndex 使用 Object_Para >> 3。
 *   - FilterConfig 将消息路由到 RX FIFO0 或 RX FIFO1。
 */
static void fdcan_filter_mask_config(FDCAN_HandleTypeDef *hfdcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    FDCAN_FilterTypeDef fdcan_filter = {0};

    assert_param(hfdcan != NULL);

    fdcan_filter.IdType = ((Object_Para & 0x02) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID);
    fdcan_filter.FilterIndex = (uint8_t)(Object_Para >> 3);
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = (((Object_Para >> 2) & 0x01) ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0);

    // FDCAN 的 FilterID1/FilterID2 使用整ID值；用作简单掩码匹配（这里做直观映射）
    fdcan_filter.FilterID1 = ID;
    fdcan_filter.FilterID2 = Mask_ID;

    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);
}

/**
 * @brief 初始化 FDCAN 总线并启用接收中断
 */
void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, FDCAN_Call_Back Callback_Function)
{
    // 启动 FDCAN
    HAL_FDCAN_Start(hfdcan);

    // 激活接收中断（FIFO0 和 FIFO1 新消息中断）
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    if (hfdcan->Instance == FDCAN1)
    {
        FDCAN1_Manage_Object.FDCAN_Handler = hfdcan;
        FDCAN1_Manage_Object.Callback_Function = Callback_Function;

        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(0) | FDCAN_FIFO_0 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(1) | FDCAN_FIFO_1 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        FDCAN2_Manage_Object.FDCAN_Handler = hfdcan;
        FDCAN2_Manage_Object.Callback_Function = Callback_Function;

        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(14) | FDCAN_FIFO_0 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(15) | FDCAN_FIFO_1 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        FDCAN3_Manage_Object.FDCAN_Handler = hfdcan;
        FDCAN3_Manage_Object.Callback_Function = Callback_Function;

        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(28) | FDCAN_FIFO_0 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
        fdcan_filter_mask_config(hfdcan, FDCAN_FILTER(29) | FDCAN_FIFO_1 | FDCAN_STDID | FDCAN_DATA_TYPE, 0, 0);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @param IDR_type 扩展帧
 * @param Frame_type 发送帧类型
 * @return uint8_t 执行状态
 */
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint32_t ID, uint8_t *Data, uint16_t Length, uint32_t IDR_type,uint32_t Frame_type)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    HAL_StatusTypeDef halret;

    assert_param(hfdcan != NULL);

    //----------- ID 设置（标准 / 扩展）-----------
    tx_header.Identifier = ID;
    tx_header.IdType = (IDR_type == FDCAN_STANDARD_ID) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;

    //----------- 数据帧 / 远程帧 -----------
    tx_header.TxFrameType = (Frame_type == FDCAN_REMOTE_TYPE) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

    // 设置数据长度码（使用字节数转换为 FDCAN DLC：HAL 提供宏 FDCAN_DLC_BYTES_x，但也可直接使用 helper）
    switch (Length)
    {
    case 0: tx_header.DataLength = FDCAN_DLC_BYTES_0; break;
    case 1: tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
    case 2: tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
    case 3: tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
    case 4: tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
    case 5: tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
    case 6: tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
    case 7: tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
    default:
    case 8: tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 将消息放入 FIFO/Q（HAL_FDCAN_AddMessageToTxFifoQ 返回 HAL_StatusTypeDef）
    halret = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data);

    return (uint8_t)halret; // 0 -> HAL_OK
}

/**
 * @brief 1ms 定时器回调（原 TIM_1ms_CAN_PeriodElapsedCallback）
 */
void TIM_1ms_FDCAN_PeriodElapsedCallback()
{
    FDCAN_Send_Data(&hfdcan1, 0x200, FDCAN1_0x200_Tx_Data, 8, FDCAN_STANDARD_ID,FDCAN_DATA_TYPE);
}

/* -------------------------------------------------------------------------- */
/*                        中断回调：FIFO0 / FIFO1                             */
/* -------------------------------------------------------------------------- */

/**
 * HAL FDCAN 接收 FIFO0 回调（HAL 会在内部触发此回调）
 *
 * 注意：此处实现与原 HAL_CAN_RxFifo0MsgPendingCallback 相对应的处理逻辑。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64]; // FDCAN 支持更大 payload（FD），但当前使用经典 CAN 模式（8字节）, 所以只用了8字节

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
        return;

    if (hfdcan->Instance == FDCAN1)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            // 拷贝数据（最多 8 bytes，原逻辑）
            memcpy(FDCAN1_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN1_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN1_Manage_Object.Callback_Function)
                FDCAN1_Manage_Object.Callback_Function(&FDCAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            memcpy(FDCAN2_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN2_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN2_Manage_Object.Callback_Function)
                FDCAN2_Manage_Object.Callback_Function(&FDCAN2_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            // 拷贝数据（最多 8 bytes，原逻辑）
            memcpy(FDCAN3_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN3_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN3_Manage_Object.Callback_Function)
                FDCAN3_Manage_Object.Callback_Function(&FDCAN3_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * HAL FDCAN 接收 FIFO1 回调
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    //判断程序初始化完成
    if (init_finished == false)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)
        return;

    if (hfdcan->Instance == FDCAN1)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
        {
            memcpy(FDCAN1_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN1_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN1_Manage_Object.Callback_Function)
                FDCAN1_Manage_Object.Callback_Function(&FDCAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
        {
            memcpy(FDCAN2_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN2_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN2_Manage_Object.Callback_Function)
                FDCAN2_Manage_Object.Callback_Function(&FDCAN2_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
        {
            memcpy(FDCAN3_Manage_Object.Rx_Buffer.Data, rx_data, 8);
            FDCAN3_Manage_Object.Rx_Buffer.Header = rx_header;
            if (FDCAN3_Manage_Object.Callback_Function)
                FDCAN3_Manage_Object.Callback_Function(&FDCAN3_Manage_Object.Rx_Buffer);
        }
    }
}
