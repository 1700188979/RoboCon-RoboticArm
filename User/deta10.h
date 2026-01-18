//
// Created by Lenovo on 25-11-26.
//

#ifndef DETA10_H
#define DETA10_H

#include "main.h"
#include "string.h"

//末端陀螺仪
#define FRAME_HEADER 0xFC
#define FRAME_TAIL   0xFD

typedef uint64_t timestamp_t;
typedef struct
{
    float RollSpeed;
    float PitchSpeed;
    float HeadingSpeed;
    float Roll;
    float Pitch;
    float Heading;
    float Q1;
    float Q2;
    float Q3;
    float Q4;
    timestamp_t Timestamp;
}FDILink_AHRSData_Packet_t;

void IMU_get_measer(uint8_t *buffer, uint8_t length);
extern FDILink_AHRSData_Packet_t AHRSData; //末端陀螺仪，控制水平
extern float IMUdata[4];


#endif //DETA10_H
