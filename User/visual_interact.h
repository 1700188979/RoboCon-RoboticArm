//
// Created by Lenovo on 25-12-13.
//

#ifndef VISUAL_RECEIVE_H
#define VISUAL_RECEIVE_H

#include "stm32h7xx_hal.h"

#define VISUAL_FRAME_HEADER 0x0A
#define VISUAL_FRAME_TAIL 0x6B

#define VISUAL_UART huart10
/**
 * @brief openmv数据接收处理函数
 *     0x0A x y z 0x6b
 */
#pragma pack(1)
typedef struct Visual_RxConvert_Typedef1
{
    uint8_t head;
    float x;
    float y;
    float z;
    uint8_t tail;
}Visual_RxConvert_Typedef1;
#pragma pack()

/**
 * @brief 接收视觉信息
 *     0x0A flag 0x6b
 */
#pragma pack(1)
typedef struct Visual_RxConvert_Typedef2
{
    uint8_t head;
    uint8_t flag;
    uint8_t tail;
}Visual_RxConvert_Typedef2;
#pragma pack()

void Visual_Get_Convert1(uint8_t *buffer, uint8_t length);
void Visual_Get_Convert2(uint8_t *buffer, uint8_t length);
void Transmit_Visual_Transformation_Matrix(float t1, float t2, float t3, float t4);

extern Visual_RxConvert_Typedef1 Data_Visual_Receive1;
extern Visual_RxConvert_Typedef2 Data_Visual_Receive2;
extern uint8_t VS_Receive_flag;
extern uint8_t Vofa_Receive_flag;
extern uint8_t text_rx_finish[2];
extern uint8_t a;
#endif //VISUAL_RECEIVE_H
