//
// Created by Lenovo on 25-11-26.
//

#include "deta10.h"

float IMUdata[4];					//用IMUdata[0]测水平
FDILink_AHRSData_Packet_t AHRSData; //末端陀螺仪，控制水平
/**
 * @brief 陀螺仪读取
 */
void IMU_get_measer(uint8_t *buffer, uint8_t length)
{
    if (length != 56) return; // AHRS 数据帧固定长度为 56

    if (buffer[0] != FRAME_HEADER || buffer[55] != FRAME_TAIL)
        return;

    uint8_t cmd_type = buffer[1];
    uint8_t data_len = buffer[2];  // 应该是 48
    uint8_t crc8 = buffer[4];
    uint16_t crc16 = (buffer[5] << 8) | buffer[6];

    if (data_len != 48 || cmd_type != 0x41)
        return;
    memcpy(&AHRSData,buffer+7,sizeof(FDILink_AHRSData_Packet_t));
    // 保存原始数据至IMUdata
    IMUdata[0] = AHRSData.Pitch;    // 原始俯仰角（绝对角度）
    IMUdata[1] = AHRSData.Roll;     // 原始横滚角（绝对角度）
    IMUdata[2] = AHRSData.Heading;  // 原始航向角（绝对角度，无偏移）
    IMUdata[3] = 1;                 // 数据有效标志（保持原逻辑）
}




