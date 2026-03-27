/**
 * @file tsk_config_and_callback.cpp
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *
 */

/* Includes ------------------------------------------------------------------*/
// .Cpp file:
#include "tsk_config_and_callback.h"         // 包含自定义的头文件

// 添加 .cpp 的头文件（通常放置在此处）
#include "drv_tim_h7.h"
#include "dvc_motor_dji_h7.h"
#include "drv_fdcan_h7.h"
#include "dvc_vesc_h7.h"
#include "robotic_arm.h"
#include "robotic_recycling.h"

// .C file:
extern "C" {              // 使用 C 语言链接方式
#include "UART_DMA_h7.h"
#include "vofa.h"
#include "vofa_setting.h"
#include "stdio.h"
#include "stdint.h"
#include "arm_math.h"
#include "deta10.h"
#include "visual_interact.h"
}
// 全局初始化完成标志位
bool init_finished = false;
// 机械臂类
Class_Robotic_arm RoboticArm;
Class_Robotic_Recycling RoboticArmRecycling;

uint8_t a=0;

/**
 * @brief FDCAN1 回调函数
 * @param FDCAN_RxMessage FDCAN1 收到的消息
 */
void Device_FDCAN1_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
	// FDCAN: 标准帧 = FDCAN_STANDARD_ID，扩展帧 = FDCAN_EXTENDED_ID
	if (FDCAN_RxMessage->Header.IdType == FDCAN_STANDARD_ID)//判断是否为标准帧 —— RM2006/3508/6020,DM电机
	{

		switch (FDCAN_RxMessage->Header.Identifier)
		{
			case 0x05:
			{
				RoboticArm.arm_motor1.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}
			case 0x06:
			{
				RoboticArm.arm_motor2.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}
			case 0x07:
			{
				RoboticArm.arm_motor3.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}
			case 0x08:
			{
				RoboticArm.arm_motor4.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}
			default:
				break;
		}
	}
	else if (FDCAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID)//判断是否为扩展帧（VESC）
	{
		switch (FDCAN_RxMessage->Header.Identifier)
		{
			default:
				break;
		}
	}
}

/**
 * @brief FDCAN2 回调函数
 * @param FDCAN_RxMessage FDCAN2 收到的消息
 */
void Device_FDCAN2_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
	// FDCAN: 标准帧 = FDCAN_STANDARD_ID，扩展帧 = FDCAN_EXTENDED_ID
	if (FDCAN_RxMessage->Header.IdType == FDCAN_STANDARD_ID)//判断是否为标准帧 —— RM2006/3508/6020,DM电机
	{

		switch (FDCAN_RxMessage->Header.Identifier)
		{
			case 0x201:
			{
				RoboticArmRecycling.recycling_motor_elevator.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}

			case 0x202:
			{
				RoboticArmRecycling.recycling_motor_transport.FDCAN_RxCpltCallback(FDCAN_RxMessage->Data);
				break;
			}
			default:
				break;
		}
	}
	else if (FDCAN_RxMessage->Header.IdType == FDCAN_EXTENDED_ID)//判断是否为扩展帧（VESC）
	{
		switch (FDCAN_RxMessage->Header.Identifier)
		{
			default:
				break;
		}
	}
}

void Device_FDCAN3_Callback(Struct_FDCAN_Rx_Buffer *FDCAN_RxMessage)
{
	switch (FDCAN_RxMessage->Header.Identifier)
	{
		case (0x00):
		{
			break;
		}
	}
}

/**
 * @brief TIM5任务回调函数,每1ms进入一次
 *
 */
void Task1ms_TIM5_Callback()
{
	if (init_finished==0)
		return;

	// 1Hz
	static int alive_mod1000 = 0;
	alive_mod1000++;
	if (alive_mod1000 == 1000)
	{
		alive_mod1000 = 0;

	}

	//100Hz
	static int interaction_mod10 = 0;
	interaction_mod10++;
	if (interaction_mod10 == 10)
	{
		interaction_mod10 = 0;

		// Transmit_Visual_Transformation_Matrix(RoboticArm.DH_arm_motor[0].Next_Angle,RoboticArm.DH_arm_motor[1].Next_Angle,RoboticArm.DH_arm_motor[2].Next_Angle,RoboticArm.DH_arm_motor[3].Next_Angle);
		// Transmit_Visual_Transformation_Matrix(RoboticArm.DH_arm_motor[0].Now_Angle,RoboticArm.DH_arm_motor[1].Now_Angle,RoboticArm.DH_arm_motor[2].Now_Angle,RoboticArm.DH_arm_motor[3].Now_Angle);
	}

	//100Hz
	static int data_mod1 = 0;
	data_mod1++;
	if (data_mod1 == 10)
	{
		data_mod1 = 0;

		RoboticArm.Robotic_TIM_10ms_PeriodElapsedCallback();
		RoboticArmRecycling.Robotic_Recycling_TIM_10ms_PeriodElapsedCallback();
	}
}
/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
	HAL_Delay(1000);

	// FDCAN 总线初始化
	FDCAN_Init(&hfdcan1, Device_FDCAN1_Callback);
	FDCAN_Init(&hfdcan2, Device_FDCAN2_Callback);
	FDCAN_Init(&hfdcan3, Device_FDCAN3_Callback);

    // 定时器初始化
    TIM_Init(&htim5, Task1ms_TIM5_Callback);

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim5);

	//蜂鸣器
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	// TIM12->CCR2=3000;

	//机械臂初始化
	RoboticArm.Init();

	//运输装置初始化
	RoboticArmRecycling.Init();

	HAL_Delay(1000);

	RoboticArm.Robotic_Motor_Get();

	RoboticArm.Set_Target_point(-0.01,0.33,0.83,5,Facing_Forward,Fifth_Order);		//初始位置
	if (RoboticArm.Joint_Space_Preprocessing()==1)
	{
		RoboticArm.path_finish_flag=0;
	}
	RoboticArmRecycling.Motion_Control_Elevator(1);
	// UART初始化
	UART_DMA_Receive_init(&huart1, buffer_receive_1, buffer_receive_length_1);//配置串口1接收VOFA+
	UART_DMA_Receive_init(&huart7, buffer_receive_7, buffer_receive_length_7);//配置串口7接收陀螺仪
	UART_DMA_Receive_init(&huart10, buffer_receive_10, buffer_receive_length_10);//配置串口10接收摄像头

	HAL_Delay(4000);//陀螺仪上电5s
	//初始化完成
	init_finished = true;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
    while (1)
    {
    	RoboticArm.Robotic_Main();

		tempFloat[0]=RoboticArm.DH_arm_motor[0].Next_Angle*180/PI;
		tempFloat[1]=RoboticArm.DH_arm_motor[0].Next_Omega*180/PI;
		tempFloat[2]=RoboticArm.DH_arm_motor[1].Next_Angle*180/PI;
		tempFloat[3]=RoboticArm.DH_arm_motor[1].Next_Omega*180/PI;
		tempFloat[4]=RoboticArm.DH_arm_motor[2].Next_Angle*180/PI;
		tempFloat[5]=RoboticArm.DH_arm_motor[2].Next_Omega*180/PI;
		tempFloat[6]=RoboticArm.DH_arm_motor[0].Now_Angle*180/PI;
		tempFloat[7]=RoboticArm.DH_arm_motor[1].Now_Angle*180/PI;
		tempFloat[8]=RoboticArm.DH_arm_motor[2].Now_Angle*180/PI;
    	tempFloat[9]=RoboticArm.DH_arm_motor[3].Now_Angle*180/PI;
    	tempFloat[10]=RoboticArmRecycling.recycling_motor_elevator.Get_Now_Angle()*180/PI;
    	tempFloat[11]=RoboticArm.arm_motor4.Get_Now_Omega();
		tempFloat[12]=IMUdata[1]*180/PI-2;
		tempFloat[13]=Vofa_Slider1;
		tempFloat[14]=Vofa_Slider2;
		tempFloat[15]=Vofa_Slider3;
    	tempFloat[16]=a;

		Vofa_Transmit(&huart1,17);
    }
}

void H7_TIM_CALLBACK(TIM_HandleTypeDef *htim)
{
	H7_HAL_TIM_PeriodElapsedCallback(htim);
}

/* 测试用代码1 单电机 */
// RoboticArm.DH_arm_motor[0].Next_Angle=Vofa_Slider1;
// RoboticArm.DH_arm_motor[1].Next_Angle=Vofa_Slider2;
// RoboticArm.DH_arm_motor[2].Next_Angle=Vofa_Slider3;
// RoboticArm.DH_arm_motor[0].Next_Omega=(0);
// RoboticArm.DH_arm_motor[1].Next_Omega=(0);
// RoboticArm.DH_arm_motor[2].Next_Omega=(0);

/* 测试用代码2 吸盘 */
// RoboticArm.Air_Pump(1);
// HAL_Delay(2000);
// RoboticArm.Air_Pump(0);
// HAL_Delay(2000);


