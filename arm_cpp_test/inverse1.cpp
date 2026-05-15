#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.14159265f
// 定义4x4矩阵结构体
typedef struct {
    float Matrix[4][4];
} Matrix4x4;
Matrix4x4 Error_Result;

float Cal_theta2(float theta3,float K1,float K2,float a2,float a3);
float Angle_Normalization(float theta)
{
	if (fabs(theta)<1e-4)	return 1e-4;
	if (theta>PI)			return theta-2*PI;
	if (theta<-PI)			return theta+2*PI;
	return theta;
}
//范围为(-PI,PI]
//theta2求解
//此步骤可以得出s2和c2，故可确定唯一解theta2
float Cal_theta2(float theta3,float K1,float K2,float a2,float a3)
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

float Cal_theta3(float K1,float K2,float a2,float a3,int* err_flag)
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
		return 0.114514f;
	}
		return acos(C);
}
Matrix4x4 Calculate_kinematics_Inverse(float x,float y,float z, int* err_flag)
{
	float a1=0;// 关节2参数
	float a2=0.39;// 关节3参数
	float a3=0.36;// 关节4参数
	float a4=0.14;// 关节4参数
	Matrix4x4 result;
	*err_flag=0;	//初始化
	//theta1求解
	//解1
	result.Matrix[0][0]=atan2(y,x);
    if (fabs(result.Matrix[0][0]) < 1e-4)      //避免除零
        result.Matrix[0][0] = 1e-4;
	result.Matrix[1][0]=result.Matrix[0][0];
	//解2
	result.Matrix[2][0]=Angle_Normalization(result.Matrix[0][0]+PI);
	result.Matrix[3][0]=result.Matrix[2][0];

	float K1_1,K2_1,K1_2,K2_2;
	K1_1=x/cos(result.Matrix[0][0])-a1;
	K2_1=a4+z;
	K1_2=x/cos(result.Matrix[2][0])-a1;
	K2_2=a4+z;

	//theta3求解
	//解1_1	解1_2
	result.Matrix[0][2]=Cal_theta3(K1_1,K2_1,a2,a3,err_flag);
	result.Matrix[1][2]=-result.Matrix[0][2];
	//解2_1 解2_2
	result.Matrix[2][2]=Cal_theta3(K1_2,K2_2,a2,a3,err_flag);
	result.Matrix[3][2]=-result.Matrix[2][2];
	if (*err_flag==1)			// 超范围，返回错误
	{
		return Error_Result;
	}

	//theta2求解
	//此步骤可以得出s2和c2，故可确定唯一解theta2
	result.Matrix[0][1]=Cal_theta2(result.Matrix[0][2],K1_1,K2_1,a2,a3);
	result.Matrix[1][1]=Cal_theta2(result.Matrix[1][2],K1_1,K2_1,a2,a3);
	result.Matrix[2][1]=Cal_theta2(result.Matrix[2][2],K1_2,K2_2,a2,a3);
	result.Matrix[3][1]=Cal_theta2(result.Matrix[3][2],K1_2,K2_2,a2,a3);

	//theta4求解:theta4=PI/2-theta2-theta3
	result.Matrix[0][3]=Angle_Normalization(-PI/2-result.Matrix[0][1]-result.Matrix[0][2]);
	result.Matrix[1][3]=Angle_Normalization(-PI/2-result.Matrix[1][1]-result.Matrix[1][2]);
	result.Matrix[2][3]=Angle_Normalization(-PI/2-result.Matrix[2][1]-result.Matrix[2][2]);
	result.Matrix[3][3]=Angle_Normalization(-PI/2-result.Matrix[3][1]-result.Matrix[3][2]);

	return result;
}

// 示例使用
int main() {
    int* flag;
    Matrix4x4 result=Calculate_kinematics_Inverse(0.21205,0.21205,0.46540,flag);
    // 打印矩阵验证（此时应为单位矩阵）
    if(*flag==0)
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            printf("%.4f ", result.Matrix[i][j]*180/PI);
        }
        printf("\n");
    }
    else
        printf("gg\n");
    return 0;
}