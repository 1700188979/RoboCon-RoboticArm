//
// Created by Lenovo on 26-3-23.
//

#include "robotic_recycling.h"

// 添加 .cpp 的头文件（通常放置在此处）
#include "drv_tim_h7.h"
#include "dvc_motor_dji_h7.h"
#include "drv_fdcan_h7.h"

// .C file:
extern "C" {              // 使用 C 语言链接方式
#include "UART_DMA_h7.h"
#include "vofa.h"
#include "vofa_setting.h"
#include "stdio.h"
#include "stdint.h"
#include "arm_math.h"
}

void Class_Robotic_Recycling::Init()
{
	recycling_motor_elevator.Init(&hfdcan2,Motor_DJI_ID_0x201,Motor_DJI_Control_Method_ANGLE);
	recycling_motor_elevator.PID_Omega.Init(2.0f, 0.00f, 0.0f, 0.0f, 10.0f * PI, 16.0f * PI, 0.001f,0.01);
	recycling_motor_elevator.PID_Angle.Init(5.0f, 0.000f, 0.001f, 0.0f, 10.0f * PI, 16.0f * PI, 0.001f,0.01);

	recycling_motor_transport.Init(&hfdcan2,Motor_DJI_ID_0x202,Motor_DJI_Control_Method_ANGLE);
	recycling_motor_transport.PID_Omega.Init(2.0f, 0.00f, 0.0f, 0.0f, 10.0f * PI, 16.0f * PI, 0.001f,0.01);
	recycling_motor_transport.PID_Angle.Init(5.0f, 0.000f, 0.001f, 0.0f, 10.0f * PI, 16.0f * PI, 0.001f,0.01);
}

void Class_Robotic_Recycling::Recycling_Set_Target_point(Robotic_Recycling_Motor_Type* Type, float distance, float T,
	Enum_Robotic_Recycling_Motor_Function_Type Function_Type, Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve Order_OF)
{
	if (T <= 0) {
		return ;
	}
	Type->Function_Type=Function_Type;
	Type->Order_OF=Order_OF;
	if (Function_Type==Elevator)
	{
		Type->Targrt_Angle=distance*Gain_Coefficient_Elevator;
	}
	else if (Function_Type==Transport)
	{
		Type->Targrt_Angle=distance*Gain_Coefficient_Transport;
	}
	Type->T=T;
	Type->N=T/RECYCLING_Shortest_Interval;
}

/**
 * @brief 清理目标点
 */
void Class_Robotic_Recycling::Recycling_Clear_Path_Planning(Robotic_Recycling_Motor_Type* Type)
{
	//清除a_quintic
	for (int j=0;j<6;j++)
	{
			Type->a_quintic[j] = 0;
	}
	Type->Order_OF=RECYCLEING_Fifth_Order;
	Type->Targrt_Angle=0;
	Type->FunTimes=0;
	Type->T=0;
	Type->N=0;
}

/**
 * @brief 计算该段路径规划区间角度方程的六个参数
 * @param T:该段路程设计的总时间
 * @param theta0:初始角度
 * @param thetaf:结束角度
 * @param a
 */
void Class_Robotic_Recycling::Recycling_Joint_Space_Quintic_Cal_Via_Para(float T,float theta0,float thetaf,Enum_Order_OF_Robotic_Recycling_Path_Planning_Curve Target_Order_Num,float a[6])
{
	if (T <= 0) {
		return ;// 检查时间间隔有效性
	}

	// 计算高次项系数
	float T2=T*T;
	float T3=T2*T;
	float T4=T3*T;
	float T5=T4*T;

	if (Target_Order_Num==RECYCLEING_Fifth_Order)
	{
		float H=thetaf-theta0;
		// 计算误差项
		float temp1=20.0f*H;
		float temp2=-30.0f*H;
		float temp3=12.0f*H;
		// 计算各系数
		a[0]=theta0;														// a0
		a[1]=0;																// a1
		a[2]=0;																// a2
		a[3]=temp1*0.5f/T3;													// a3
		a[4]=temp2*0.5f/T4;													// a4
		a[5]=temp3*0.5f/T5;													// a5
	}

	if (Target_Order_Num==RECYCLEING_Third_Order)
	{
		float H=thetaf-theta0;
		a[0]=theta0;														// a0
		a[1]=0;																// a1
		a[2]=3.0f*H/T2;														// a2
		a[3]=-2.0f*H/T3;													// a3
		a[4]=0;																// a4
		a[5]=0;																// a5
	}
}

inline void Class_Robotic_Recycling::Recycling_Joint_Space_Preprocessing(Robotic_Recycling_Motor_Type* Type)
{
	if (Type->Function_Type==Elevator)
	{
		Recycling_Joint_Space_Quintic_Cal_Via_Para(Type->T,recycling_motor_elevator.Get_Now_Angle(),Type->Targrt_Angle,Type->Order_OF,Type->a_quintic);
	}
	else if (Type->Function_Type==Transport)
	{
		Recycling_Joint_Space_Quintic_Cal_Via_Para(Type->T,recycling_motor_transport.Get_Now_Angle(),Type->Targrt_Angle,Type->Order_OF,Type->a_quintic);
	}
	Type->path_finish_flag=0;
}

/**
 * @brief 当前的目标角度、速度及加速度的计算
 */
void Class_Robotic_Recycling::Recycling_Joint_Space_Via_Path_Planning(Robotic_Recycling_Motor_Type* Type)
{
	float theta_now=Type->FunTimes*RECYCLING_Shortest_Interval;
	float theta_now2=theta_now*theta_now;
	float theta_now3=theta_now2*theta_now;
	float theta_now4=theta_now3*theta_now;
	float theta_now5=theta_now4*theta_now;
	float Angle=Type->a_quintic[0]+theta_now*Type->a_quintic[1]+theta_now2*Type->a_quintic[2]+theta_now3*Type->a_quintic[3]+theta_now4*Type->a_quintic[4]+theta_now5*Type->a_quintic[5];
	float Speed=Type->a_quintic[1]+2*Type->a_quintic[2]*theta_now+3*Type->a_quintic[3]*theta_now2+4*Type->a_quintic[4]*theta_now3+5*Type->a_quintic[5]*theta_now4;
	float Acceleration=2*Type->a_quintic[2]+6*Type->a_quintic[3]*theta_now+12*Type->a_quintic[4]*theta_now2+20*Type->a_quintic[5]*theta_now3;

	//存储目标值
	Type->Next_Angle=Angle;
	Type->Next_Omega=Speed;
	Type->Next_Acceleration=Acceleration;
}

/**
 * @brief 传递当前
 * @param FunTimes:已经过的时间单位个数 由定时器计数输入（从1到N）
 */
void Class_Robotic_Recycling::Recycling_Joint_Space_Path_Planning(Robotic_Recycling_Motor_Type* Type)
{
	if (Type->FunTimes<=Type->N)
	{
		Recycling_Joint_Space_Via_Path_Planning(Type);
	}
	else
	{
		Type->FunTimes=0;
		Type->path_finish_flag=1;
		Recycling_Clear_Path_Planning(Type);
	}
}



/**
 * @brief 该机构两个电机的PID计算
 */
void Class_Robotic_Recycling::Robotic_Recycling_TIM_10ms_PeriodElapsedCallback()
{
	if (recycling_para_elevator.path_finish_flag==0)
	{
		recycling_para_elevator.FunTimes++;
		//路径规划计算
		Recycling_Joint_Space_Path_Planning(&recycling_para_elevator);
	}

	if (recycling_para_transport.path_finish_flag==0)
	{
		recycling_para_transport.FunTimes++;
		//路径规划计算
		Recycling_Joint_Space_Path_Planning(&recycling_para_transport);
	}

	recycling_motor_elevator.Set_Target_Omega(recycling_para_elevator.Next_Omega);
	recycling_motor_elevator.Set_Target_Angle(recycling_para_elevator.Next_Angle);
	recycling_motor_elevator.TIM_Calculate_PeriodElapsedCallback();

	recycling_motor_transport.Set_Target_Omega(recycling_para_transport.Next_Omega);
	recycling_motor_transport.Set_Target_Angle(recycling_para_transport.Next_Angle);
	recycling_motor_transport.TIM_Calculate_PeriodElapsedCallback();

	TIM_1ms_FDCAN_PeriodElapsedCallback();
}

void Class_Robotic_Recycling::Motion_Control_Elevator(uint8_t state)
{
	if (state==0)
	{
		return;
	}
	if (state==1)
	{
		Recycling_Set_Target_point(&recycling_para_elevator,PI*3,2,Elevator,RECYCLEING_Fifth_Order);
		Recycling_Joint_Space_Preprocessing(&recycling_para_elevator);
		recycling_para_elevator.path_finish_flag=0;
	}
}

void Class_Robotic_Recycling::Motion_Control_Transport(uint8_t state)
{
	if (state==0)
	{
		return;
	}
	if (state==1)
	{
		Recycling_Set_Target_point(&recycling_para_transport,PI,2,Transport,RECYCLEING_Fifth_Order);
		Recycling_Joint_Space_Preprocessing(&recycling_para_transport);
		recycling_para_transport.path_finish_flag=0;
	}
}

