//
// Created by Lenovo on 25-12-13.
//

#include "visual_interact.h"

#include <math.h>
#include "string.h"
#include "usart.h"

uint8_t text_rx_finish[2]={VISUAL_FRAME_HEADER,VISUAL_FRAME_TAIL};
Visual_RxConvert_Typedef Data_Visual_Receive;
uint8_t VS_Receive_flag=0;
uint8_t Vofa_Receive_flag=0;

float a_visual=0.22f;
uint8_t Send_Data[50];
/*
 * 规定 0x0A (float)x (float)y (float)z 0x6B
 */
void Visual_Get_Convert(uint8_t *buffer, uint8_t length)
{
    if (length != 14) return; // 通信协议约定数据帧固定长度为 14

    if (buffer[0] != VISUAL_FRAME_HEADER || buffer[13] != VISUAL_FRAME_TAIL)
        return;
    memcpy(&Data_Visual_Receive,buffer,sizeof(Visual_RxConvert_Typedef));

    VS_Receive_flag=1;
}

/*
 * 规定 0x0A (float)x (float)y (float)z 0x6B
 */
void Transmit_Visual_Transformation_Matrix(float t1, float t2, float t3, float t4)
{
    float result[12];
    float a1=0.0f;
    float a2=0.39f;
    float a3=0.36f;
    float a4=0.22f;

    float t23 = t2 + t3;       // t2 + t3
    float t234 = t2 + t3 + t4; // t2 + t3 + t4
    float c1 = cosf(t1);
    float c2 = cosf(t2);
    float c23 = cosf(t23);
    float c234 = cosf(t234);
    float s1 = sinf(t1);
    float s2 = sinf(t2);
    float s23 = sinf(t23);
    float s234 = sinf(t234);

    // 计算旋转矩阵部分（3x3）
    result[0] = c1 * c234;              // R11
    result[3] = -c1 * s234;             // R12
    result[6] = s1;                     // R13
    result[1] = s1 * c234;              // R21
    result[4] = -s1 * s234;             // R22
    result[7] = -c1;                    // R23
    result[2] = s234;                   // R31
    result[5] = c234;                   // R32
    result[8] = 0.0f;                   // R33

    // 计算平移向量部分（3x1）
    result[9] = c1*(a1 + a2*c2 + a3*c23 + a4*c234);     // Px
    result[10] = s1*(a1 + a2*c2 + a3*c23 + a4*c234);    // Py
    result[11] = a2*s2 + a3*s23 + a4*s234;              // Pz

    memcpy(Send_Data+1, (uint8_t*)result, 48);
    Send_Data[0]=VISUAL_FRAME_HEADER;
    Send_Data[49]=VISUAL_FRAME_TAIL;
    HAL_UART_Transmit(&VISUAL_UART,Send_Data, sizeof(Send_Data),0xff);
}
