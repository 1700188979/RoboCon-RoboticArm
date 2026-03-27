//
// Created by Lenovo on 26-3-23.
//

#ifndef ROBOTIC_RECYCLING_H
#define ROBOTIC_RECYCLING_H

#include "dvc_motor_dji_h7.h"

#define Gain_Coefficient_Elevator 1             // 升降电机实际距离对应的弧度
#define Gain_Coefficient_Transport 1            // 运输电机实际距离对应的弧度

#define RECYCLING_POS_MAX_NUM 10

#define RECYCLING_Shortest_Interval 0.01f          // 10ms控制一次

enum Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve
{
    RECYCLEING_Third_Order = 0,
    RECYCLEING_Fifth_Order = 1
};

enum Enum_Robotic_Recycling_Motor_Function_Type
{
    Elevator = 0,
    Transport = 1
};

typedef struct Robotic_Recycling_Motor_Type
{
    float Targrt_Angle = 0;

    float T = 0;

    float N = 0;                    // N=T/RECYCLING_Shortest_Interval

    float a_quintic[6] = {0};             // 方程参数

    float FunTimes = 0;

    float Next_Angle = 0;

    float Next_Omega = 0;

    float Next_Acceleration = 0;

    uint8_t path_finish_flag = 1;

    Enum_Robotic_Recycling_Motor_Function_Type Function_Type;

    Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve Order_OF = RECYCLEING_Fifth_Order;

}Robotic_Recycling_Motor_Type;

class Class_Robotic_Recycling
{
public:

    void Init();

    void Recycling_Set_Target_point(Robotic_Recycling_Motor_Type* Type, float distance, float T,
        Enum_Robotic_Recycling_Motor_Function_Type Function_Type, Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve Order_OF);

    void Recycling_Joint_Space_Quintic_Cal_Via_Para(float T, float theta0, float thetaf, Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve Target_Order_Num, float a[6]);

    void Recycling_Clear_Path_Planning(Robotic_Recycling_Motor_Type* Type);

    inline void Recycling_Joint_Space_Preprocessing(Robotic_Recycling_Motor_Type* Type);

    void Recycling_Joint_Space_Via_Path_Planning(Robotic_Recycling_Motor_Type* Type);

    void Recycling_Joint_Space_Path_Planning(Robotic_Recycling_Motor_Type* Type);

    void Robotic_Recycling_TIM_10ms_PeriodElapsedCallback();

    void Motion_Control_Elevator(uint8_t state);

    void Motion_Control_Transport(uint8_t state);

    Robotic_Recycling_Motor_Type recycling_para_elevator;      // 控制升降

    Robotic_Recycling_Motor_Type recycling_para_transport;     // 控制运输

    Class_Motor_DJI_C610 recycling_motor_elevator;

    Class_Motor_DJI_C610 recycling_motor_transport;

};

#endif //ROBOTIC_RECYCLING_H
