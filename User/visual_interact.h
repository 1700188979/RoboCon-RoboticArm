//
// Created by Lenovo on 25-12-13.
//

#ifndef VISUAL_RECEIVE_H
#define VISUAL_RECEIVE_H

#include "stm32h7xx_hal.h"

#define VISUAL_FRAME_HEADER 0x0A
#define VISUAL_FRAME_TAIL 0x6B

#define VISUAL_UART huart3
/**
 * @brief openmv数据接收处理函数
 *     0x0A x y z 0x6b
 */
#pragma pack(1)
typedef struct Visual_RxConvert_Typedef
{
    uint8_t head;
    float x;
    float y;
    float z;
    uint8_t tail;
}Visual_RxConvert_Typedef;
#pragma pack()

void Visual_Get_Convert(uint8_t *buffer, uint8_t length);
void Transmit_Visual_Transformation_Matrix(float t1, float t2, float t3, float t4);

extern Visual_RxConvert_Typedef Data_Visual_Receive;
extern uint8_t VS_Receive_flag;
extern uint8_t Vofa_Receive_flag;
extern uint8_t text_rx_finish[2];
#endif //VISUAL_RECEIVE_H
