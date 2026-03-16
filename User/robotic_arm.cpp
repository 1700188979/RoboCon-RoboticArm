//
// Created by Lenovo on 25-11-14.
//

#include "robotic_arm.h"

// 添加 .cpp 的头文件（通常放置在此处）
#include "drv_tim_h7.h"
#include "dvc_motor_dji_h7.h"
#include "dvc_motor_dm_h7.h"
#include "drv_fdcan_h7.h"

// .C file:
extern "C" {              // 使用 C 语言链接方式
#include "UART_DMA_h7.h"
#include "vofa.h"
#include "vofa_setting.h"
#include "stdio.h"
#include "stdint.h"
#include "arm_math.h"
#include "visual_interact.h"
#include "deta10.h"
}

/**
 * @brief 计算T1*T2并返回结果矩阵
 */
Matrix4x4 matrix_multiply(Matrix4x4 T1, Matrix4x4 T2)
{
	Matrix4x4 result;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.Matrix[i][j] = 0.0f;
			for (int k = 0; k < 4; k++) {
				result.Matrix[i][j] += T1.Matrix[i][k] * T2.Matrix[k][j];
			}
		}
	}
	return result;
}

/**
 * @brief 角度归一化
 */
inline float Angle_Normalization(float theta)
{
	if (fabs(theta)<1e-4)	return 1e-4;
	if (theta>PI)			return theta-2*PI;
	if (theta<-PI)			return theta+2*PI;
	return theta;
}

/**
 * @brief 机械臂初始化
 */
void Class_Robotic_arm::Init()
{
	// 成员变量初始化
    arm_motor1.Init(&hfdcan1,0x05,0x01,Motor_DM_Control_Method_NORMAL_MIT,12.5,10,28,0.9);
	arm_motor2.Init(&hfdcan1,0x06,0x02,Motor_DM_Control_Method_NORMAL_MIT,12.5,10,28,1);
	arm_motor3.Init(&hfdcan1,0x07,0x03,Motor_DM_Control_Method_NORMAL_MIT,12.5,10,28,0.9);
	arm_motor4.Init(&hfdcan1,0x08,0x04,Motor_DM_Control_Method_NORMAL_MIT,12.5,10,28,0.8);
	arm_motor1.FDCAN_Send_Enter();
	arm_motor2.FDCAN_Send_Enter();
	arm_motor3.FDCAN_Send_Enter();
	arm_motor4.FDCAN_Send_Enter();
	Robotic_Motor_Get();			//得到初始角度
	HAL_Delay(200);
	/* PID参数初始化 */
	arm_motor1.Set_K_D(1);
	arm_motor1.Set_K_P(5);
	arm_motor2.Set_K_D(1);
	arm_motor2.Set_K_P(100);
	arm_motor3.Set_K_D(5);
	arm_motor3.Set_K_P(100);
	arm_motor4.Set_K_D(1);
	arm_motor4.Set_K_P(0);
	arm_motor3.Set_Control_Torque(10);

	//末端水平控制器
	Horizontal_Controller.Init(35,18,0,0,0,5 );
	Horizontal_Controller.Set_Target(0);

	Init_Maxtrix();
	Clear_Path_Planning();
}

/**
 * @brief DH参数初始化
 */
void Class_Robotic_arm::Init_Maxtrix()
{
	// 转换矩阵初始化
	DH_arm_motor[0].a=0.0f;
	DH_arm_motor[0].d=0.0f;
	DH_arm_motor[0].alpha=PI/2;
	DH_arm_motor[0].Now_Angle=0;
	DH_arm_motor[0].bias=0;
	DH_arm_motor[0].MAX_Angle=PI;
	DH_arm_motor[0].MIN_Angle=0;
	DH_arm_motor[0].Rotation=Reverse_Rotation;
	DH_arm_motor[0].Reduction_Ratio=2.5;
	DH_arm_motor[0].Next_Angle=0;

	DH_arm_motor[1].a=0.280f;
	DH_arm_motor[1].d=0.0f;
	DH_arm_motor[1].alpha=0;
	DH_arm_motor[1].Now_Angle=65.0f/180.0f*PI;
	DH_arm_motor[1].bias=0;
	DH_arm_motor[1].MAX_Angle=90.0f/180.0f*PI;
	DH_arm_motor[1].MIN_Angle=50.0f/180.0f*PI;
	DH_arm_motor[1].Rotation=Forward_Rotation;
	DH_arm_motor[1].Reduction_Ratio=2.0;
	DH_arm_motor[1].Next_Angle=65.0f/180.0f*PI;

	DH_arm_motor[2].a=0.560f;
	DH_arm_motor[2].d=0.0f;
	DH_arm_motor[2].alpha=0.0f;
	DH_arm_motor[2].Now_Angle=-45.0f/180.0f*PI;
	DH_arm_motor[2].bias=0;
	DH_arm_motor[2].MAX_Angle=0.0f;
	DH_arm_motor[2].MIN_Angle=-PI*0.5f;
	DH_arm_motor[2].Rotation=Forward_Rotation;
	DH_arm_motor[2].Reduction_Ratio=1.0;
	DH_arm_motor[2].Next_Angle=-45.0f/180.0f*PI;

	DH_arm_motor[3].a=0.220f;//length
	DH_arm_motor[3].d=0.0f;
	DH_arm_motor[3].alpha=0.0f;
	DH_arm_motor[3].Now_Angle=-45.0f/180.0f*PI;
	DH_arm_motor[3].bias=0;
	DH_arm_motor[3].MAX_Angle=60.0f/180.0f*PI;
	DH_arm_motor[3].MIN_Angle=-90.0f/180.0f*PI;
	DH_arm_motor[3].Rotation=Reverse_Rotation;
	DH_arm_motor[3].Reduction_Ratio=1.0;
	DH_arm_motor[3].Next_Angle=-45.0f/180.0f*PI;

	robot_cal_T(&DH_arm_motor[0]);
	robot_cal_T(&DH_arm_motor[1]);
	robot_cal_T(&DH_arm_motor[2]);
	robot_cal_T(&DH_arm_motor[3]);
}

/**
 * @brief 计算单个转换矩阵（连乘矩阵公式已展开，此函数暂时无用）
 * @param alphai
 * @param ai
 * @param di
 * @param Angle
 */
void Class_Robotic_arm::robot_cal_T(Transformation_Matrix_Para *Matrix_Para)
{
	float alpha=Matrix_Para->alpha;
	float a=Matrix_Para->a;
	float d=Matrix_Para->d;
	float theta=Matrix_Para->Now_Angle;

	float ca = cosf(alpha);
	float sa = sinf(alpha);
	float ct = cosf(theta);
	float st = sinf(theta);

	Matrix_Para->Matrix.Matrix[0][0] = ct;
	Matrix_Para->Matrix.Matrix[0][1] = -st*ca;
	Matrix_Para->Matrix.Matrix[0][2] = st*sa;
	Matrix_Para->Matrix.Matrix[0][3] = a*ct;

	Matrix_Para->Matrix.Matrix[1][0] = st;
	Matrix_Para->Matrix.Matrix[1][1] = ct * ca;
	Matrix_Para->Matrix.Matrix[1][2] = -ct*sa;
	Matrix_Para->Matrix.Matrix[1][3] = a * st;

	Matrix_Para->Matrix.Matrix[2][0] = 0;
	Matrix_Para->Matrix.Matrix[2][1] = sa;
	Matrix_Para->Matrix.Matrix[2][2] = ca;
	Matrix_Para->Matrix.Matrix[2][3] = d;

	Matrix_Para->Matrix.Matrix[3][0] = 0;
	Matrix_Para->Matrix.Matrix[3][1] = 0;
	Matrix_Para->Matrix.Matrix[3][2] = 0;
	Matrix_Para->Matrix.Matrix[3][3] = 1;
}

/**
 * @brief 5个矩阵连乘（连乘矩阵公式已展开，此函数暂时无用）
 * @param T1，T2，T3，T4
 */
inline Matrix4x4 Class_Robotic_arm::robot_cal_4T(Matrix4x4 T1,Matrix4x4 T2,Matrix4x4 T3,Matrix4x4 T4)
{
	return matrix_multiply(matrix_multiply(matrix_multiply(T1, T2), T3), T4);
}

/**
 * @brief 计算总变换矩阵（基于通用表达式）
 */
Matrix4x4 Class_Robotic_arm::Calculate_kinematics_Transform()
{
    Matrix4x4 result;
	float a1=DH_arm_motor[0].a, t1=DH_arm_motor[0].Now_Angle;  // 关节1参数
	float a2=DH_arm_motor[1].a, t2=DH_arm_motor[1].Now_Angle;  // 关节2参数
	float a3=DH_arm_motor[2].a, t3=DH_arm_motor[2].Now_Angle;  // 关节3参数
	float a4=DH_arm_motor[3].a, t4=DH_arm_motor[3].Now_Angle;  // 关节4参数

	float t23 = t2 + t3;       // t2 + t3
	float t234 = t2 + t3 + t4; // t2 + t3 + t4
	float c1 = cos(t1);
	float c2 = cos(t2);
	float c23 = cos(t23);
	float c234 = cos(t234);
	float s1 = sin(t1);
	float s2 = sin(t2);
	float s23 = sin(t23);
	float s234 = sin(t234);

	// 计算旋转矩阵部分（3x3）
	result.Matrix[0][0] = c1 * c234;       // R11
	result.Matrix[0][1] = -c1 * s234;      // R12
	result.Matrix[0][2] = s1;              // R13
	result.Matrix[1][0] = s1 * c234;       // R21
	result.Matrix[1][1] = -s1 * s234;      // R22
	result.Matrix[1][2] = -c1;             // R23
	result.Matrix[2][0] = s234;            // R31
	result.Matrix[2][1] = c234;            // R32
	result.Matrix[2][2] = 0.0f;            // R33

	// 计算平移向量部分（3x1）
	result.Matrix[0][3] = c1*(a1 + a2*c2 + a3*c23 + a4*c234);  // Px
	result.Matrix[1][3] = s1*(a1 + a2*c2 + a3*c23 + a4*c234);  // Py
	result.Matrix[2][3] = a2*s2 + a3*s23 + a4*s234;            // Pz

	// 第四行固定值
	result.Matrix[3][0] = 0.0f;
	result.Matrix[3][1] = 0.0f;
	result.Matrix[3][2] = 0.0f;
	result.Matrix[3][3] = 1.0f;

	return result;
}

/**
 * @brief 逆运动学解算的预处理，帮助判断特殊情况。返回1代表有问题，0则无问题
 * @param 目标点坐标x,y,z
 */
uint8_t Class_Robotic_arm::Kinematics_Inverse_Preprocessing(float x, float y, float z)
{
	float L=DH_arm_motor[0].a+DH_arm_motor[1].a+DH_arm_motor[2].a+DH_arm_motor[3].a;
	//超限问题
	if (x*x+y*y+z*z>=L*L)
	{
		return 1;
	}

	return 0;
}


/**
 * @brief 逆运动学解算，角度范围为[-PI,PI]
 * @param 目标点坐标x,y,z
 * @param err_flag:解算超限标志
 */
Matrix4x4 Class_Robotic_arm::Calculate_kinematics_Inverse(float x, float y, float z, uint8_t* err_flag, Pose_Orientation Attitude)
{
	float a1=DH_arm_motor[0].a;	// 关节2参数
	float a2=DH_arm_motor[1].a;	// 关节3参数
	float a3=DH_arm_motor[2].a;	// 关节4参数
	float a4=DH_arm_motor[3].a;	// 关节4参数
	Matrix4x4 result;

	// z+=Suction_Distence; // 加上吸盘的高度

	err_flag[0]=Kinematics_Inverse_Preprocessing(x,y,z);	//预处理判断特殊情况
	err_flag[1]=err_flag[0];
	if (err_flag[0]==1)			// 超范围，返回错误
	{
		return Error_Result;
	}
	// theta1求解
	float Singularity_flag=0;
	// 解1
	result.Matrix[0][0]=atan2(y,x);
	if (fabs(result.Matrix[0][0]) < 1e-4 || fabs(result.Matrix[0][0] - PI) < 1e-4 || fabs(result.Matrix[0][0] + PI) < 1e-4)
	{
		// 处理奇异点	奇异点为0 PI -PI
		Singularity_flag=1;
		if (fabs(result.Matrix[0][0]) < 1e-4)				result.Matrix[0][0] = 0;
		else if (fabs(result.Matrix[0][0] - PI) < 1e-4)	result.Matrix[0][0] = PI;
		else if (fabs(result.Matrix[0][0] + PI) < 1e-4)	result.Matrix[0][0] = -PI;
	}
	result.Matrix[1][0]=result.Matrix[0][0];
	// 解2
	result.Matrix[2][0]=Angle_Normalization(result.Matrix[0][0]+PI);
	result.Matrix[3][0]=result.Matrix[2][0];

	float K1_1=0.0f,K2_1=0.0f,K1_2=0.0f,K2_2=0.0f;
	if (Attitude==Facing_Downward)
	{
		if (Singularity_flag == 0)
		{
			K1_1 = y / sin(result.Matrix[0][0]) - a1;
			K2_1 = a4 + z;
			K1_2 = y / sin(result.Matrix[2][0]) - a1;
			K2_2 = a4 + z;
		}
		else if (Singularity_flag == 1)
		{
			K1_1 = x / cos(result.Matrix[0][0]) - a1;
			K2_1 = a4 + z;
			K1_2 = x / cos(result.Matrix[2][0]) - a1;
			K2_2 = a4 + z;
		}

	}
	else if (Attitude==Facing_Forward)
	{
		if (Singularity_flag == 0)
		{
			K1_1 = y / sin(result.Matrix[0][0]) - a1 - a4;
			K2_1 = z;
			K1_2 = y / sin(result.Matrix[2][0]) - a1 - a4;
			K2_2 = z;
		}
		else if (Singularity_flag == 1)
		{
			K1_1 = x / cos(result.Matrix[0][0]) - a1 - a4;
			K2_1 = z;
			K1_2 = x / cos(result.Matrix[2][0]) - a1 - a4;
			K2_2 = z;
		}
	}

	// theta3求解
	// 解1_1	解1_2
	result.Matrix[0][2]=Cal_theta3(K1_1,K2_1,a2,a3,err_flag);
	result.Matrix[1][2]=-result.Matrix[0][2];
	// 解2_1 解2_2
	result.Matrix[2][2]=Cal_theta3(K1_2,K2_2,a2,a3,err_flag+1);
	result.Matrix[3][2]=-result.Matrix[2][2];
	if (err_flag[0]==1&&err_flag[1]==1)			// 超范围，返回错误
	{
		return Error_Result;
	}

	// theta2求解
	// 此步骤可以得出s2和c2，故可确定唯一解theta2
	if (err_flag[0]==0)
	{
		result.Matrix[0][1]=Cal_theta2(result.Matrix[0][2],K1_1,K2_1,a2,a3);
		result.Matrix[1][1]=Cal_theta2(result.Matrix[1][2],K1_1,K2_1,a2,a3);
	}
	if (err_flag[1]==0)
	{
		result.Matrix[2][1]=Cal_theta2(result.Matrix[2][2],K1_2,K2_2,a2,a3);
		result.Matrix[3][1]=Cal_theta2(result.Matrix[3][2],K1_2,K2_2,a2,a3);
	}

	if (Attitude==Facing_Downward)
	{
		// theta4求解:theta4=PI/2-theta2-theta3
		if (err_flag[0]==0)
		{
			result.Matrix[0][3]=Angle_Normalization(-PI*0.5f-result.Matrix[0][1]-result.Matrix[0][2]);
			result.Matrix[1][3]=Angle_Normalization(-PI*0.5f-result.Matrix[1][1]-result.Matrix[1][2]);
		}
		if (err_flag[1]==0)
		{
			result.Matrix[2][3]=Angle_Normalization(-PI*0.5f-result.Matrix[2][1]-result.Matrix[2][2]);
			result.Matrix[3][3]=Angle_Normalization(-PI*0.5f-result.Matrix[3][1]-result.Matrix[3][2]);
		}
	}
	else if (Attitude==Facing_Forward)
	{
		// theta4求解:theta4=-theta2-theta3
		if (err_flag[0]==0)
		{
			result.Matrix[0][3]=Angle_Normalization(-result.Matrix[0][1]-result.Matrix[0][2]);
			result.Matrix[1][3]=Angle_Normalization(-result.Matrix[1][1]-result.Matrix[1][2]);
		}
		if (err_flag[1]==0)
		{
			result.Matrix[2][3]=Angle_Normalization(-result.Matrix[2][1]-result.Matrix[2][2]);
			result.Matrix[3][3]=Angle_Normalization(-result.Matrix[3][1]-result.Matrix[3][2]);
		}
	}

	return result;
}

/**
 * @brief theta2求解，此步骤可以得出s2和c2，故可确定唯一解theta2
 */
float Class_Robotic_arm::Cal_theta2(float theta3,float K1,float K2,float a2,float a3)
{
	float A,B;
	float c2,s2;
	float theta2_11;
	A=-a3*sin(theta3);
	B=a2+a3*cos(theta3);
	c2=(K2-B*K1/A)/(-A-B*B/A);
	s2=(K1-B*c2)/A;
	//特殊情况:s2=0
	if (fabs(s2)<1e-4)
	{
		if (1-c2<1e-4)
			theta2_11=1e-4;			//c2=1
		else
			theta2_11=PI-(1e-4);			//c2=-1
	}
	//特殊情况:c2=0
	else if (fabs(c2)<1e-4)
	{
		if (1-s2<1e-4)
			theta2_11=PI/2-(1e-4);			//s2=1
		else
			theta2_11=-PI/2+(1e-4);		//s2=-1
	}
	else if (c2>1e-4)					//t在一四象限
		theta2_11=asin(s2);
	else if (c2<-1e-4&&s2>1e-4)			//t在第二象限
		theta2_11=acos(c2);
	else if (c2<-1e-4&&s2<-1e-4)			//t在第三象限
		theta2_11=-acos(c2);
	return theta2_11;
}

/**
 * @brief theta3求解
 */
float Class_Robotic_arm::Cal_theta3(float K1,float K2,float a2,float a3,uint8_t* err_flag)
{
	float C=(K1*K1+K2*K2-a2*a2-a3*a3)/(2*a2*a3);
	if (fabs(C-1)<1e-4)		//C=1 角度为0
	{
		return 1e-4;
	}
	if (fabs(C+1)<1e-4)	//C=-1 角度为PI
	{
		return PI-(1e-4);
	}
	if (C>1)
	{
		*err_flag=1;
		return 11.4514f;
	}
		return acos(C);
}

/**
 * @brief Choose_Best_Angle_Group的预处理
 * @param Angle_Group:解算出的四组角度
 * @param err_num:错误的组数
 * @param err_group_flag:某组错误的标志
 */
void Class_Robotic_arm::Choose_Best_Angle_Group_Processing(Matrix4x4 Angle_Group,uint8_t* err_num,uint8_t err_group_flag[4])
{
	uint8_t err_angle_flag=0;
	for (int i=0;i<4;i++)
	{
		err_angle_flag=0;
		for (int j=0;j<4;j++)
		{
			if (Angle_Group.Matrix[i][j]>DH_arm_motor[j].MAX_Angle||Angle_Group.Matrix[i][j]<DH_arm_motor[j].MIN_Angle)
			{
				err_angle_flag=1;
				(*err_num)++;
				break;
			}
		}
		err_group_flag[i]=err_angle_flag;
	}
}

/**
 * @brief 通过比较与当前角度值的差，筛选出逆运动学解算出的四组角度中最佳的一组，返回其序号
 * @param Angle_Group:解算出的四组角度
 */
uint8_t Class_Robotic_arm::Choose_Best_Angle_Group(Matrix4x4 Angle_Group, float Last_Best_Group[4], uint8_t* err_flag)
{
	uint8_t flag=0;
	uint8_t err_group_flag[4]={0};		//0为有效
	uint8_t err_num[1]={0};
	Choose_Best_Angle_Group_Processing(Angle_Group,err_num,err_group_flag);
	if (*err_num==4)	//本次解算无效
	{
		*err_flag=1;
		return 0xFF;		//无效返回
	}

	if (*err_num==0)
	{
		if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
		{
			//最佳组在1、2组中
			if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[0][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[1][1]))
			{
				//选择第1组解
				return 0;
			}
			else
			{
				//选择第2组解
				return 1;
			}
		}
		else
		{
			//最佳组在3、4组中
			if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[2][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[3][1]))
			{
				//选择第3组解
				return 2;
			}
			else
			{
				//选择第4组解
				return 3;
			}
		}
	}

	if (*err_num==1)				// 无效组个数为1
	{
		if (err_group_flag[0]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 1;
			else
			{
				if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[2][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[3][1]))
				{
					//选择第3组解
					return 2;
				}
				else
				{
					//选择第4组解
					return 3;
				}
			}
		}
		if (err_group_flag[1]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 0;
			else
			{
				if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[2][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[3][1]))
				{
					//选择第3组解
					return 2;
				}
				else
				{
					//选择第4组解
					return 3;
				}
			}
		}
		if (err_group_flag[2]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
			{
				if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[0][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[1][1]))
				{
					//选择第1组解
					return 0;
				}
				else
				{
					//选择第2组解
					return 1;
				}
			}
			else
			{
				return 3;
			}
		}
		if (err_group_flag[3]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
			{
				if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[0][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[1][1]))
				{
					//选择第1组解
					return 0;
				}
				else
				{
					//选择第2组解
					return 1;
				}
			}
			else
			{
				return 2;
			}
		}
	}

	if (*err_num==2)
	{
		if (err_group_flag[0]==1&&err_group_flag[1]==1)
		{
			if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[2][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[3][1]))
				flag=2;
			else
				flag=3;
		}
		if (err_group_flag[0]==1&&err_group_flag[2]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 1;
			else
				return 3;
		}
		if (err_group_flag[0]==1&&err_group_flag[3]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 1;
			else
				return 2;
		}
		if (err_group_flag[1]==1&&err_group_flag[2]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 0;
			else
				return 3;
		}
		if (err_group_flag[1]==1&&err_group_flag[3]==1)
		{
			if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
				return 0;
			else
				return 2;
		}
		if (err_group_flag[2]==1&&err_group_flag[3]==1)
		{
			if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[0][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[1][1]))
				return 0;
			else
				return 1;
		}
	}

	if (*err_num==3)
	{
		for (int i=0;i<4;i++)
		{
			if (err_group_flag[i]==0)	return i;	// 第i组为有效组
		}
	}
	return flag;
}

/**
 * @brief 计算该段路径规划区间角度方程的六个参数
 * @param T:该段路程设计的总时间
 * @param theta0:初始角度
 * @param d_theta0:初始角速度
 * @param dd_theta0:初始角加速度
 * @param thetaf:结束角度
 * @param d_thetaf:结束角速度
 * @param dd_thetaf:结束角加速度
 * @param a
 */
void Class_Robotic_arm::Joint_Space_Quintic_Cal_Via_Para(float T,float theta0, float d_theta0, float dd_theta0,
	float thetaf, float d_thetaf, float dd_thetaf,float a[6],Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Target_Order_Num)
{
	if (T <= 0) {
		return ;// 检查时间间隔有效性
	}

	if (Target_Order_Num==Fifth_Order)
	{
		// 计算高次项系数
		float T2=T*T;
		float T3=T2*T;
		float T4=T3*T;
		float T5=T4*T;
		float H=thetaf-theta0;
		// 计算误差项
		float temp1=20.0f*H-(8.0f*d_thetaf+12.0f*d_theta0)*T-(3.0f*dd_theta0-dd_thetaf)*T2;
		float temp2=-30.0f*H+(14.0f*d_thetaf+16.0f*d_theta0)*T+(3.0f*dd_theta0-2.0f*dd_thetaf)*T2;
		float temp3=12.0f*H-6.0f*(d_thetaf+d_theta0)*T+(dd_thetaf-dd_theta0)*T2;
		// 计算各系数
		a[0]=theta0;														// a0
		a[1]=d_theta0;														// a1
		a[2]=dd_theta0/2;													// a2
		a[3]=temp1*0.5f/T3;													// a3
		a[4]=temp2*0.5f/T4;													// a4
		a[5]=temp3*0.5f/T5;													// a5
	}

	if (Target_Order_Num==Third_Order)
	{
		float H=thetaf-theta0;
		a[0]=theta0;														// a0
		a[1]=d_theta0;														// a1
		a[2]=(3.0f*H/T-2.0f*d_theta0-d_thetaf)/T;							// a2
		a[3]=(-2.0f*H/T+d_theta0+d_thetaf)/(T*T);							// a3
		a[4]=0;																// a4
		a[5]=0;																// a5
	}
}

/**
 * @brief 当前的目标角度的计算
 * @param FunTimes:已经过的时间单位个数 由定时器计数输入（从1到N）
 * @param Ordinal_Num:电机序号
 * @param a[6]:方程的六个参数
 */
void Class_Robotic_arm::Joint_Space_Via_Path_Planning(uint8_t Ordinal_Num, float a[6])
{
	float theta_now=FunTimes*Shortest_Interval;
	float theta_now2=theta_now*theta_now;
	float theta_now3=theta_now2*theta_now;
	float theta_now4=theta_now3*theta_now;
	float theta_now5=theta_now4*theta_now;
	float Angle=a[0]+theta_now*a[1]+theta_now2*a[2]+theta_now3*a[3]+theta_now4*a[4]+theta_now5*a[5];
	float Speed=a[1]+2*a[2]*theta_now+3*a[3]*theta_now2+4*a[4]*theta_now3+5*a[5]*theta_now4;
	float Acceleration=2*a[2]+6*a[3]*theta_now+12*a[4]*theta_now2+20*a[5]*theta_now3;

	//存储目标值
	DH_arm_motor[Ordinal_Num].Next_Angle=Angle;
	DH_arm_motor[Ordinal_Num].Next_Omega=Speed;
	DH_arm_motor[Ordinal_Num].Next_Acceleration=Acceleration;
}

/**
 * @brief 各点参量自动生成
 * @param Target_Angle_Matrix:num个逆解矩阵
 * @param best_flag:每个Target_Angle_Matrix对应四个角度的最佳解的标号
 * @param temp_d:每个点理想的速度分量
 * @param temp_dd:每个点理想的加速度分量
 */
void Class_Robotic_arm::Joint_Space_Automatic_Para_Generation(uint8_t best_flag[])
{
	//起点特殊处理
	for (int j=0;j<4;j++)
	{
		if (Target_Angle_Matrix[0].Matrix[best_flag[0]][j] > DH_arm_motor[j].Now_Angle &&
			Target_Angle_Matrix[0].Matrix[best_flag[0]][j] > Target_Angle_Matrix[1].Matrix[best_flag[1]][j])
			//此点为极大值点
		{
			temp_d[0][j] = 0;
			temp_dd[0][j] = -Turning_Acceleration;
		}
		else if (Target_Angle_Matrix[0].Matrix[best_flag[0]][j] < DH_arm_motor[j].Now_Angle &&
				 Target_Angle_Matrix[0].Matrix[best_flag[0]][j] < Target_Angle_Matrix[1].Matrix[best_flag[1]][j])
			//此点为极小值点
		{
			temp_d[0][j] = 0;
			temp_dd[0][j] = Turning_Acceleration;
		}
		else
		{
			temp_d[0][j] = ((Target_Angle_Matrix[1].Matrix[best_flag[1]][j] - Target_Angle_Matrix[0].Matrix[best_flag[0]][j]) / position[1][3] +
							(Target_Angle_Matrix[0].Matrix[best_flag[0]][j] - DH_arm_motor[j].Now_Angle) / position[0][3]) * 0.5;
			temp_dd[0][j] = Turning_Acceleration * (signbit(temp_d[0][j]) ? -1.0f : 1.0f);
		}
	}
	//终点特殊处理
	temp_d[acc_num-1][0]=0;
	temp_dd[acc_num-1][0]=0;
	temp_d[acc_num-1][1]=0;
	temp_dd[acc_num-1][1]=0;
	temp_d[acc_num-1][2]=0;
	temp_dd[acc_num-1][2]=0;
	temp_d[acc_num-1][3]=0;
	temp_dd[acc_num-1][3]=0;
	if (acc_num>=3)
	{
		for (int i=1;i<acc_num-1;i++)
		{
			for (int j=0;j<4;j++)
			{
				//角1
				if (Target_Angle_Matrix[i].Matrix[best_flag[i]][j] > Target_Angle_Matrix[i-1].Matrix[best_flag[i-1]][j] &&
					Target_Angle_Matrix[i].Matrix[best_flag[i]][j] > Target_Angle_Matrix[i+1].Matrix[best_flag[i+1]][j])
					//此点为极大值点
				{
					temp_d[i][j] = 0;
					temp_dd[i][j] = -Turning_Acceleration;
				}
				else if (Target_Angle_Matrix[i].Matrix[best_flag[i]][j] < Target_Angle_Matrix[i-1].Matrix[best_flag[i-1]][j] &&
						 Target_Angle_Matrix[i].Matrix[best_flag[i]][j] < Target_Angle_Matrix[i+1].Matrix[best_flag[i+1]][j])
					//此点为极小值点
				{
					temp_d[i][j] = 0;
					temp_dd[i][j] = Turning_Acceleration;
				}
				else
				{
					temp_d[i][j] = ((Target_Angle_Matrix[i+1].Matrix[best_flag[i+1]][j] - Target_Angle_Matrix[i].Matrix[best_flag[i]][j]) / position[i+1][3] +
									(Target_Angle_Matrix[i].Matrix[best_flag[i]][j] - Target_Angle_Matrix[i-1].Matrix[best_flag[i-1]][j]) / position[i][3]) * 0.5;
					temp_dd[i][j] = Turning_Acceleration * (signbit(temp_d[i][j]) ? -1.0f : 1.0f);
				}
			}
		}
	}
}

/**
 * @brief 对路径规划的预处理:进行所有目标点的逆解算，选取各个点的姿态最佳解，算出到下一个点过程的参量a
 * 返回值1表示规划成功，0为规划失败
 */
uint8_t Class_Robotic_arm::Joint_Space_Preprocessing()
{
	// Matrix4x4 Target_Angle_Matrix[acc_num];
	uint8_t err_flag[3]={0};		//每次在Calculate_kinematics_Inverse中初始化
	float Last_Best_Group[4]={DH_arm_motor[0].Now_Angle,DH_arm_motor[1].Now_Angle,DH_arm_motor[2].Now_Angle,DH_arm_motor[3].Now_Angle};
	//先筛选出最佳角度方案
	for (int i=0;i<acc_num;i++)
	{
		Target_Angle_Matrix[i]=Calculate_kinematics_Inverse(position[i][0],position[i][1],position[i][2],err_flag,Attitude[i]);	//角度解算
		if (err_flag[0]==1&&err_flag[1]==1)	//本次解算无效
		{
			Clear_Path_Planning();
			return 0;
		}
		best_flag[i]=Choose_Best_Angle_Group(Target_Angle_Matrix[i],Last_Best_Group,err_flag+2);		//筛选最佳组
		if (err_flag[2]==1)	//本次解算无效
		{
			Clear_Path_Planning();
			return 0;
		}
		for (int j=0;j<4;j++)
		{
			Last_Best_Group[j]=Target_Angle_Matrix[i].Matrix[best_flag[i]][j];
		}

		if (acc_num==1)
		{
			for (int j=0;j<4;j++)
			{
				Joint_Space_Quintic_Cal_Via_Para(position[0][3], DH_arm_motor[j].Now_Angle, 0, 0,
					Target_Angle_Matrix[0].Matrix[best_flag[0]][j], 0, 0, a_quintic[0][j],Order_Num[0]);	//计算参数
			}
			now_num=0;
			return 1;
		}
	}

	//自动生成每个点理想的速度和加速度分量
	// float temp_d[acc_num][4],temp_dd[acc_num][4];
	Joint_Space_Automatic_Para_Generation(best_flag);

	//再进行参数计算
	for (int j=0;j<4;j++)	//第一次特殊处理
	{
		Joint_Space_Quintic_Cal_Via_Para(position[0][3], DH_arm_motor[j].Now_Angle, temp_d[0][j], temp_dd[0][j],
			Target_Angle_Matrix[0].Matrix[best_flag[0]][j], 0, 0, a_quintic[0][j],Order_Num[0]);//计算参数
	}
	for (int i=1;i<acc_num;i++)
	{
		for (int j=0;j<4;j++)
		{
			Joint_Space_Quintic_Cal_Via_Para(position[i][3], Target_Angle_Matrix[i-1].Matrix[best_flag[i-1]][j], temp_d[i-1][j], temp_dd[i-1][j],
				Target_Angle_Matrix[i].Matrix[best_flag[i]][j], temp_d[i][j], temp_dd[i][j], a_quintic[i][j],Order_Num[i]);//计算参数
		}
	}

	now_num=0;
	return 1;
}

/**
 * @brief 传递当前
 * @param FunTimes:已经过的时间单位个数 由定时器计数输入（从1到N）
 */
void Class_Robotic_arm::Joint_Space_4Angle_Group_Path_Planning()
{
	if (FunTimes<=position[now_num][4])
	{
		for (int j=0;j<4;j++)
		{
			Joint_Space_Via_Path_Planning(j,a_quintic[now_num][j]);
		}
	}
	else
	{
		now_num++;
		FunTimes=0;
		if (now_num==acc_num)
		{
			now_num=0;
			acc_num=0;
			path_finish_flag=1;
			Clear_Path_Planning();
		}
	}
}

/**
 * @brief 设置目标点
 * t最好不要小于0.4s
 */
void Class_Robotic_arm::Set_Target_point(float x, float y, float z, float t, Pose_Orientation Target_Attitude, Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Target_Order_Num)
{
	if (acc_num>POS_MAX_NUM)
		return;
	position[acc_num][0]=x;
	position[acc_num][1]=y;
	position[acc_num][2]=z;
	position[acc_num][3]=t;
	position[acc_num][4]=t/Shortest_Interval;
	Attitude[acc_num]=Target_Attitude;
	Order_Num[acc_num]=Target_Order_Num;
	acc_num++;
}

/**
 * @brief 设置目标点
 */
void Class_Robotic_arm::Clear_Path_Planning()
{
	//清除position
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		for (int j=0;j<5;j++)
		{
			position[i][j]=0;
		}
	}
	//清除Attitude
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		Attitude[i]=Facing_Forward;
	}
	//清除Order_Num
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		Order_Num[i]=Third_Order;
	}
	FunTimes=0;
	//清除a_quintic
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		for (int j=0;j<4;j++)
		{
			for(int k=0;k<6;k++)
			{
				a_quintic[i][j][k] = 0;
			}
		}
	}
	//清除best_flag
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		best_flag[i]=0;
	}
}

/**
 * @brief 设置下一个时间间隔的值
 * 路径规划下最短时间间隔的角度和角速度传递
 */
void Class_Robotic_arm::Robotic_Motor_Set()
{
	// Keep_Forearm_Horizontal();
	if (DH_arm_motor[0].Rotation==Forward_Rotation)
	{
		arm_motor1.Set_Control_Angle((DH_arm_motor[0].Next_Angle-DH_arm_motor[0].bias)*DH_arm_motor[0].Reduction_Ratio);
		arm_motor1.Set_Control_Omega(DH_arm_motor[0].Next_Omega*DH_arm_motor[0].Reduction_Ratio);
	}
	else
	{
		arm_motor1.Set_Control_Angle(-(DH_arm_motor[0].Next_Angle-DH_arm_motor[0].bias)*DH_arm_motor[0].Reduction_Ratio);
		arm_motor1.Set_Control_Omega(-(DH_arm_motor[0].Next_Omega)*DH_arm_motor[0].Reduction_Ratio);
	}

	if (DH_arm_motor[1].Rotation==Forward_Rotation)
	{
		arm_motor2.Set_Control_Angle((DH_arm_motor[1].Next_Angle-DH_arm_motor[1].bias)*DH_arm_motor[1].Reduction_Ratio);
		arm_motor2.Set_Control_Omega(DH_arm_motor[1].Next_Omega*DH_arm_motor[1].Reduction_Ratio);
	}
	else
	{
		arm_motor2.Set_Control_Angle(-(DH_arm_motor[1].Next_Angle-DH_arm_motor[1].bias)*DH_arm_motor[1].Reduction_Ratio);
		arm_motor2.Set_Control_Omega(-(DH_arm_motor[1].Next_Omega)*DH_arm_motor[1].Reduction_Ratio);
	}

	if (DH_arm_motor[2].Rotation==Forward_Rotation)
	{
		arm_motor3.Set_Control_Angle((DH_arm_motor[2].Next_Angle-DH_arm_motor[2].bias)*DH_arm_motor[2].Reduction_Ratio);
		arm_motor3.Set_Control_Omega(DH_arm_motor[2].Next_Omega*DH_arm_motor[2].Reduction_Ratio);
	}
	else
	{
		arm_motor3.Set_Control_Angle(-(DH_arm_motor[2].Next_Angle-DH_arm_motor[2].bias)*DH_arm_motor[2].Reduction_Ratio);
		arm_motor3.Set_Control_Omega(-(DH_arm_motor[2].Next_Omega)*DH_arm_motor[2].Reduction_Ratio);
	}
	if (DH_arm_motor[3].Rotation==Forward_Rotation)
	{
		arm_motor4.Set_Control_Omega(-Horizontal_Controller.Get_Out());
		arm_motor4.Set_Control_Angle(0);
	}
	else
	{
		arm_motor4.Set_Control_Omega(Horizontal_Controller.Get_Out());
		arm_motor4.Set_Control_Angle(0);

	}
	// if (DH_arm_motor[3].Rotation==Forward_Rotation)
	// {
	// 	arm_motor4.Set_Control_Angle(-IMUdata[0]*DH_arm_motor[3].Reduction_Ratio*2);
	// 	arm_motor4.Set_Control_Omega(DH_arm_motor[3].Next_Omega*DH_arm_motor[3].Reduction_Ratio);
	// }
	// else
	// {
	// 	arm_motor4.Set_Control_Angle(-(DH_arm_motor[3].Next_Angle-DH_arm_motor[3].bias)*DH_arm_motor[3].Reduction_Ratio+IMUdata[0]*DH_arm_motor[3].Reduction_Ratio);
	// 	// arm_motor4.Set_Control_Angle(-(DH_arm_motor[3].Next_Angle-DH_arm_motor[3].bias)*DH_arm_motor[3].Reduction_Ratio);
	// 	arm_motor4.Set_Control_Omega(-(DH_arm_motor[3].Next_Omega)*DH_arm_motor[3].Reduction_Ratio);
	// }
}

/**
 * @brief 读取当前值
 */
void Class_Robotic_arm::Robotic_Motor_Get()
{
	//角度
	DH_arm_motor[0].Now_Angle=arm_motor1.Get_Now_Angle()/DH_arm_motor[0].Reduction_Ratio+DH_arm_motor[0].bias;
	DH_arm_motor[1].Now_Angle=arm_motor2.Get_Now_Angle()/DH_arm_motor[1].Reduction_Ratio+DH_arm_motor[1].bias;
	DH_arm_motor[2].Now_Angle=arm_motor3.Get_Now_Angle()/DH_arm_motor[2].Reduction_Ratio+DH_arm_motor[2].bias;
	DH_arm_motor[3].Now_Angle=arm_motor4.Get_Now_Angle()/DH_arm_motor[3].Reduction_Ratio+DH_arm_motor[3].bias;
	//角速度
	DH_arm_motor[0].Now_Omega=arm_motor1.Get_Now_Omega()/DH_arm_motor[0].Reduction_Ratio;
	DH_arm_motor[1].Now_Omega=arm_motor2.Get_Now_Omega()/DH_arm_motor[1].Reduction_Ratio;
	DH_arm_motor[2].Now_Omega=arm_motor3.Get_Now_Omega()/DH_arm_motor[2].Reduction_Ratio;
	DH_arm_motor[3].Now_Omega=arm_motor4.Get_Now_Omega()/DH_arm_motor[3].Reduction_Ratio;
}

/**
 * @brief 机械臂4电机的PID计算
 */
void Class_Robotic_arm::Robotic_TIM_Send_PeriodElapsedCallback()
{
	//读取当前值并设置下一个时间间隔的值
	Robotic_Motor_Get();
	Robotic_Motor_Set();
	//电机PID计算
	arm_motor1.TIM_Send_PeriodElapsedCallback();
	arm_motor2.TIM_Send_PeriodElapsedCallback();
	arm_motor3.TIM_Send_PeriodElapsedCallback();
	arm_motor4.TIM_Send_PeriodElapsedCallback();	//arm_motor4 pid计算
}

/**
 * @brief 对路径规划的预处理:进行所有目标点的逆解算，选取各个点的姿态最佳解，算出到下一个点过程的参量a
 * 返回值1表示规划成功，0为规划失败
 */
uint8_t Class_Robotic_arm::Intime_Joint_Space_Dynamic_Tuning(float Intime_x, float Intime_y, float Intime_z, float Intime_T,
	Pose_Orientation Intime_Attitude, Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Intime_Order_Num)
{
	uint8_t err_flag[3]={0};		//每次在Calculate_kinematics_Inverse中初始化
	static float Intime_Last_Best_Group[4]={DH_arm_motor[0].Now_Angle,DH_arm_motor[1].Now_Angle,DH_arm_motor[2].Now_Angle,DH_arm_motor[3].Now_Angle};
	//先筛选出最佳角度方案
	Matrix4x4 intime_Target_Angle_Matrix=Calculate_kinematics_Inverse(Intime_x,Intime_y,Intime_z,err_flag,Intime_Attitude);	//角度解算
	if (err_flag[0]==1&&err_flag[1]==1)	//本次解算无效
	{
		Clear_Path_Planning();
		return 0;
	}
	uint8_t Intime_best_flag=Choose_Best_Angle_Group(intime_Target_Angle_Matrix,Intime_Last_Best_Group,err_flag+2);		//筛选最佳组
	if (err_flag[2]==1)	//本次解算无效
	{
		Clear_Path_Planning();
		return 0;
	}

	for (int j=0;j<4;j++)	//第一次特殊处理
	{
		Joint_Space_Quintic_Cal_Via_Para(Intime_T, Intime_Last_Best_Group[j], 0, 0,
			intime_Target_Angle_Matrix.Matrix[Intime_best_flag][j], 0, 0, Intime_a_quintic[j],Intime_Order_Num);//计算参数}
	}
	//更新上一次的最佳组
	for (int j=0;j<4;j++)
	{
		Intime_Last_Best_Group[j]=intime_Target_Angle_Matrix.Matrix[Intime_best_flag][j];
	}
	return 1;
}

/**
 * @brief 传递当前
 * @param FunTimes:已经过的时间单位个数 由定时器计数输入（从1到N）
 */
void Class_Robotic_arm::Intime_Joint_Space_4Angle_Group_Path_Planning()
{
	if (FunTimes*Shortest_Interval<=Intime_T)
	{
		for (int j=0;j<4;j++)
		{
			Joint_Space_Via_Path_Planning(j,Intime_a_quintic[j]);
		}
	}
	else
	{
		FunTimes=0;
		intime_path_finish_flag=1;
		Clear_Path_Planning();
	}
}

/*
 * @brief 多点规划模式，预先设定好多个点或连续传入多个点等待处理
 * 为避免主程序冗长，设置Multi_Point_Planning_Model对程序封装
 */
void Class_Robotic_arm::Multi_Point_Planning_Mode()
{
	//多点规划模式
	if (path_finish_flag==0)
	{
		FunTimes++;
		//路径规划计算
		Joint_Space_4Angle_Group_Path_Planning();
	}
}

/*
 * @brief 单点规划模式，通过上位机/遥控器操控机械臂
 * 为避免主程序冗长，设置Single_Point_Planning_Model对程序封装
 */
void Class_Robotic_arm::Single_Point_Planning_Mode()
{
	//单点规划模式（即时）
	if (intime_path_finish_flag==0 && Vofa_Receive_flag==1)
	{
		FunTimes++;
		if (FunTimes>=Intime_T/Shortest_Interval)
		{
			Vofa_Receive_flag=0;
		}
		//路径规划计算
		Intime_Joint_Space_4Angle_Group_Path_Planning();
	}
}

/*
 * @brief 视觉规划模式，通过摄像头数据操控机械臂
 * @brief 为避免主程序冗长，设置Visual_Planning_Mode对程序封装
 */
void Class_Robotic_arm::Visual_Planning_Mode()
{
	//视觉规划模式（即时）
	if (intime_path_finish_flag==0 && VS_Receive_flag==1)
	{
		FunTimes++;
		if (FunTimes>=Intime_T/Shortest_Interval)
		{
			VS_Receive_flag=0;
			HAL_UART_Receive_DMA(&VISUAL_UART,text_rx_finish,2);
		}
		//路径规划计算
		Intime_Joint_Space_4Angle_Group_Path_Planning();
	}
}

/*
 * @brief 单点规划模式在主函数中的程序
 * 为避免主程序冗长，设置Single_Point_Planning_Mode_Handle_Main对程序封装
 */
void Class_Robotic_arm::Single_Point_Planning_Mode_Handle_Main()
{
	//多点规划模式
	if (Vofa_Receive_flag==1&&intime_path_finish_flag==1)
	{
		Intime_x=Vofa_Slider1;
		Intime_y=Vofa_Slider2;
		Intime_z=Vofa_Slider3;
		Intime_T=Vofa_Slider4;
		if (Intime_T>0 && Intime_Joint_Space_Dynamic_Tuning(Intime_x,Intime_y,Intime_z,Intime_T,Facing_Forward,Fifth_Order) == 1)//有效坐标
		{
			intime_path_finish_flag=0;
		}
		else
		{
			Vofa_Receive_flag=0;
		}
	}
}

/*
 * @brief 视觉规划模式在主函数中的程序
 * 为避免主程序冗长，设置Visual_Planning_Mode_Handle_Main对程序封装
 */
void Class_Robotic_arm::Visual_Planning_Mode_Handle_Main()
{
	//多点规划模式
	if (VS_Receive_flag==1&&intime_path_finish_flag==1)
	{
		Intime_x=Data_Visual_Receive.x;
		Intime_y=Data_Visual_Receive.y;
		Intime_z=Data_Visual_Receive.z;
		Intime_T=0.5;
		if (Intime_T>0 && Intime_Joint_Space_Dynamic_Tuning(Intime_x,Intime_y,Intime_z,Intime_T,Facing_Forward,Fifth_Order)==1)//有效坐标
		{
			intime_path_finish_flag=0;
		}
		else
		{
			VS_Receive_flag=0;
		}
	}
}

/*
 * @brief 机械臂类定义的外部接口函数，便于整合部件时被调用使用，定时器1ms调用一次
 */
void Class_Robotic_arm::Robotic_TIM_1ms_PeriodElapsedCallback()
{
	//多点规划模式
	Multi_Point_Planning_Mode();
	//单点规划模式（即时）
	Single_Point_Planning_Mode();
	//视觉规划模式（即时）
	Visual_Planning_Mode();
	//末端水平控制PID计算
	Horizontal_Controller.Set_Now(IMUdata[0]);
	Horizontal_Controller.TIM_Calculate_PeriodElapsedCallback();
	//电机PID计算
	Robotic_TIM_Send_PeriodElapsedCallback();
}

/*
 * @brief 机械臂类定义的外部接口函数，便于整合部件时被调用使用，放在while中空闲检测即可，或每隔一段时间调用一次
 */
void Class_Robotic_arm::Robotic_Main()
{
	//单点规划模式在主函数中的程序
	Single_Point_Planning_Mode_Handle_Main();
	//视觉规划模式在主函数中的程序
	Visual_Planning_Mode_Handle_Main();
}

void Class_Robotic_arm::Air_Pump(uint8_t status)
{
	if (status == 0) //气泵关
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
	}
	else if (status == 1) //气泵开
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
	}
}

void Class_Robotic_arm::Keep_Forearm_Horizontal(uint8_t status)
{

}