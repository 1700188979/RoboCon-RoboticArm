#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.14159265f
// 定义4x4矩阵结构体
typedef struct {
    float Matrix[4][4];
} Matrix4x4;
Matrix4x4 Error_Result;
#define Shortest_Interval 0.005f                //两点之间最小时间间隔5ms
#define POS_MAX_NUM 20                          //路径规划中间点+终点共10个点
#define Turning_Acceleration 0.5f               //转向时的角加速度绝对值0.5m/(s*s)
int acc_num=0, now_num=0;        //总点数,当前点数
float position[POS_MAX_NUM][5]={{0}};           //4位分别代表x,y,z,T,N，第一个目标点存储在0位，N=T/Shortest_Interval
float a_quintic[POS_MAX_NUM][4][6]={{{0}}};     //4位分别代表x,y,z,T，第一个目标点存储在0位
float Angle[4]={0};
float FunTimes=0;
float Next_Angle=0,Next_Omega=0,Next_Acceleration=0;
int path_finish_flag=1;
float temp_d[4][4],temp_dd[4][4];
// 计算T04矩阵，结果存入result（float类型）
void calc_T04(float t1, float t2, float t3, float t4, 
              float a0, float a1, float a2, float a3, 
              Matrix4x4 *result) {
    // 合并角度（弧度）
    float t23 = t2 + t3;       // t2 + t3
    float t234 = t2 + t3 + t4; // t2 + t3 + t4
    
    // 三角函数计算（缩写：c=cos, s=sin；数字=角度组合）
    float c1 = cosf(t1);    float s1 = sinf(t1);
    float c2 = cosf(t2);    float s2 = sinf(t2);
    float c23 = cosf(t23);  float s23 = sinf(t23);
    float c234 = cosf(t234);float s234 = sinf(t234);
    
    // 计算旋转矩阵部分（3x3）
    result->Matrix[0][0] = c1 * c234;       // R11
    result->Matrix[0][1] = -c1 * s234;      // R12
    result->Matrix[0][2] = s1;              // R13
    
    result->Matrix[1][0] = s1 * c234;       // R21
    result->Matrix[1][1] = -s1 * s234;      // R22
    result->Matrix[1][2] = -c1;             // R23
    
    result->Matrix[2][0] = s234;            // R31
    result->Matrix[2][1] = c234;            // R32
    result->Matrix[2][2] = 0.0f;            // R33
    
    // 计算平移向量部分（3x1）
    result->Matrix[0][3] = a0*c1 + a1*c1*c2 + a2*c1*c23 + a3*c1*c234;  // Px
    result->Matrix[1][3] = a0*s1 + a1*s1*c2 + a2*s1*c23 + a3*s1*c234;  // Py
    result->Matrix[2][3] = a1*s2 + a2*s23 + a3*s234;                   // Pz
    
    // 第四行固定值
    result->Matrix[3][0] = 0.0f;
    result->Matrix[3][1] = 0.0f;
    result->Matrix[3][2] = 0.0f;
    result->Matrix[3][3] = 1.0f;
}

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
	float a1=0;     // 关节2参数
	float a2=0.39;  // 关节3参数
	float a3=0.36;  // 关节4参数
	float a4=0.14;  // 关节4参数
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

//计算该段路径规划区间角度方程的六个参数
void Joint_Space_Quintic_Cal_Via_Para(float T,float theta0, float d_theta0, float dd_theta0,float thetaf, float d_thetaf, float dd_thetaf,float a[6])
{
	if (T <= 0) {
		return ;// 检查时间间隔有效性
	}
	// // 计算高次项系数
	// float T2=T*T;
	// float T3=T2*T;
	// float T4=T3*T;
	// float T5=T4*T;
	// // 计算误差项
	// float delta_theta=thetaf-(theta0+d_theta0*T+dd_theta0*T2/2);
	// float delta_d_theta=d_thetaf-(d_theta0+dd_theta0*T);
	// float delta_dd_theta=dd_thetaf-dd_theta0;
	// // 计算各系数
	// a[0]=theta0;														// a0
	// a[1]=d_theta0;														// a1
	// a[2]=dd_theta0/2;													// a2
	// a[3]=10*delta_theta/T3-6*delta_d_theta/T2+delta_dd_theta/(2*T);		// a3
	// a[4]=-15*delta_theta/T4+8*delta_d_theta/T3-delta_dd_theta/(2*T2);	// a4
	// a[5]=6*delta_theta/T5-3*delta_d_theta/T4+delta_dd_theta/(2*T3);		// a5

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

void Joint_Space_Automatic_Para_Generation(Matrix4x4 Target_Angle_Matrix[], int best_flag[], float temp_d[][4],float temp_dd[][4])
{
	//起点特殊处理
	for (int j=0;j<4;j++)
	{
		if (Target_Angle_Matrix[0].Matrix[best_flag[0]][j] > Angle[j] &&
			Target_Angle_Matrix[0].Matrix[best_flag[0]][j] > Target_Angle_Matrix[1].Matrix[best_flag[1]][j])
			//此点为极大值点
		{
			temp_d[0][j] = 0;
			temp_dd[0][j] = -Turning_Acceleration;
		}
		else if (Target_Angle_Matrix[0].Matrix[best_flag[0]][j] < Angle[j] &&
				 Target_Angle_Matrix[0].Matrix[best_flag[0]][j] < Target_Angle_Matrix[1].Matrix[best_flag[1]][j])
			//此点为极小值点
		{
			temp_d[0][j] = 0;
			temp_dd[0][j] = Turning_Acceleration;
		}
		else
		{
			temp_d[0][j] = ((Target_Angle_Matrix[1].Matrix[best_flag[1]][j] - Target_Angle_Matrix[0].Matrix[best_flag[0]][j]) / position[1][3] +
							(Target_Angle_Matrix[0].Matrix[best_flag[0]][j] - Angle[j]) / position[0][3]) * 0.5;
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
void Clear_Path_Planning()
{
	//清除position
	for (int i=0;i<POS_MAX_NUM;i++)
	{
		position[i][0]=0;
		position[i][1]=0;
		position[i][2]=0;
		position[i][3]=0;
		position[i][4]=0;

	}
	acc_num=0;
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
}

int Choose_Best_Angle_Group(Matrix4x4 Angle_Group, float Last_Best_Group[4])
{
	int flag;
	//角1
	if (fabs(Last_Best_Group[0]-Angle_Group.Matrix[0][0])<=fabs(Last_Best_Group[0]-Angle_Group.Matrix[2][0]))
	{
		//最佳组在1、2组中
		//角2
		if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[0][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[1][1]))
		{
			//选择第1组解
			flag=0;
		}
		else
		{
			//选择第2组解
			flag=1;
		}
	}
	//角1
	else
	{
		//最佳组在3、4组中
		//角2
		if (fabs(Last_Best_Group[1]-Angle_Group.Matrix[2][1])<=fabs(Last_Best_Group[1]-Angle_Group.Matrix[3][1]))
		{
			//选择第3组解
			flag=2;
		}
		else
		{
			//选择第4组解
			flag=3;
		}
	}
	return flag;
}



int Joint_Space_Preprocessing()
{
	Matrix4x4 Target_Angle_Matrix[acc_num];
	int best_flag[acc_num];
	int err_flag[1];	//每次在Calculate_kinematics_Inverse中初始化
	float Last_Best_Group[4]={Angle[0],Angle[1],Angle[2],Angle[3]};
	//先筛选出最佳角度方案
	for (int i=0;i<acc_num;i++)
	{
		Target_Angle_Matrix[i]=Calculate_kinematics_Inverse(position[i][0],position[i][1],position[i][2],err_flag);	//角度解算
		if (err_flag[0]==1)//本次解算超限无效
		{
			Clear_Path_Planning();
			return 0;
		}
		best_flag[i]=Choose_Best_Angle_Group(Target_Angle_Matrix[i],Last_Best_Group);										//筛选最佳组
		for (int j=0;j<4;j++)
		{
			Last_Best_Group[j]=Target_Angle_Matrix[i].Matrix[best_flag[i]][j];
		}

		if (acc_num==1)
		{
			for (int j=0;j<4;j++)
			{
				Joint_Space_Quintic_Cal_Via_Para(position[0][3], Angle[j], 0, 0,
					Target_Angle_Matrix[0].Matrix[best_flag[0]][j], 0, 0, a_quintic[0][j]);	//计算参数
			}
			return 1;
		}
	}

	//自动生成每个点理想的速度和加速度分量
	 
	Joint_Space_Automatic_Para_Generation(Target_Angle_Matrix,best_flag,temp_d,temp_dd);

	//再进行参数计算
	for (int j=0;j<4;j++)	//第一次特殊处理
	{
		Joint_Space_Quintic_Cal_Via_Para(position[0][3], Angle[j], 0, 0,
			Target_Angle_Matrix[0].Matrix[best_flag[0]][j], 0, 0, a_quintic[0][j]);//计算参数
	}
	for (int i=1;i<acc_num;i++)
	{
		for (int j=0;j<4;j++)
		{
			Joint_Space_Quintic_Cal_Via_Para(position[i][3], Target_Angle_Matrix[i-1].Matrix[best_flag[i-1]][j], temp_d[i-1][j], temp_dd[i-1][j],
				Target_Angle_Matrix[i].Matrix[best_flag[i]][j], temp_d[i][j], temp_dd[i][j], a_quintic[i][j]);//计算参数
		}
	}

	return 1;
}

void Joint_Space_Via_Path_Planning(int Ordinal_Num, float a[6])
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
	Next_Angle=Angle;
	Next_Omega=Speed;
	Next_Acceleration=Acceleration;
}

void Joint_Space_4Angle_Group_Path_Planning()
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
		}
	}
}

void Set_Target_point(float x, float y, float z, float t)
{
	if (acc_num>POS_MAX_NUM)
		return;
	position[acc_num][0]=x;
	position[acc_num][1]=y;
	position[acc_num][2]=z;
	position[acc_num][3]=t;
	position[acc_num][4]=t*200;
	acc_num++;
}

// 示例使用
int main() {
    /*-------------------------------------------------------正运动学----------------------------------------------------*/
    // 输入参数（t2=90°=pi/2弧度）
    float t1[4]; 
    t1[0]= 89.0f*M_PI/180;
    t1[1]= 89.0f*M_PI/180;
    t1[2]= 89.0f*M_PI/180;
    t1[3]= 89.0f*M_PI/180;
    float t2[4]; 
    t2[0]= 89.0f*M_PI/180;
    t2[1]= 89.0f*M_PI/180;
    t2[2]= 89.0f*M_PI/180;
    t2[3]= 89.0f*M_PI/180;
    float t3[4]; 
    t3[0]= -89.0f*M_PI/180;
    t3[1]= -89.0f*M_PI/180;
    t3[2]= -89.0f*M_PI/180;
    t3[3]= -89.0f*M_PI/180;
    float t4[4]; 
    t4[0]=Angle_Normalization(-PI/2-t2[0]-t3[0]);
    t4[1]=Angle_Normalization(-PI/2-t2[1]-t3[1]);
    t4[2]=Angle_Normalization(-PI/2-t2[2]-t3[2]);
    t4[3]=Angle_Normalization(-PI/2-t2[3]-t3[3]);

	t4[0]=Angle_Normalization(-PI-t2[0]-t3[0]);
    t4[1]=Angle_Normalization(-PI-t2[1]-t3[1]);
    t4[2]=Angle_Normalization(-PI-t2[2]-t3[2]);
    t4[3]=Angle_Normalization(-PI-t2[3]-t3[3]);

    float a0 = 0.0f, a1 = 0.28f, a2 = 0.56f, a3 = 0.22f;

    // 定义result存储矩阵（float类型）
    Matrix4x4 fdresult[4];
    
    // 计算T04
    calc_T04(t1[0], t2[0], t3[0], t4[0], a0, a1, a2, a3, &fdresult[0]);
    calc_T04(t1[1], t2[1], t3[1], t4[1], a0, a1, a2, a3, &fdresult[1]);
    calc_T04(t1[2], t2[2], t3[2], t4[2], a0, a1, a2, a3, &fdresult[2]);
    calc_T04(t1[3], t2[3], t3[3], t4[3], a0, a1, a2, a3, &fdresult[3]);
    
    // 打印结果（验证）
    printf("T04矩阵（float存储）：\n");
    for (int k = 0; k < 4; k++){
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                printf("%8.5f ", fdresult[k].Matrix[i][j]);
            }
            printf("\n");
        }
        printf("\n");
    }
    

    /*-------------------------------------------------------逆运动学----------------------------------------------------*/
    int flag[4];
    Matrix4x4 result[4];
    result[0]=Calculate_kinematics_Inverse(fdresult[0].Matrix[0][3],fdresult[0].Matrix[1][3],fdresult[0].Matrix[2][3],flag);
    result[1]=Calculate_kinematics_Inverse(fdresult[1].Matrix[0][3],fdresult[1].Matrix[1][3],fdresult[1].Matrix[2][3],flag+1);
    result[2]=Calculate_kinematics_Inverse(fdresult[2].Matrix[0][3],fdresult[2].Matrix[1][3],fdresult[2].Matrix[2][3],flag+2);
    result[3]=Calculate_kinematics_Inverse(fdresult[3].Matrix[0][3],fdresult[3].Matrix[1][3],fdresult[3].Matrix[2][3],flag+3);
    // 打印矩阵验证（此时应为单位矩阵）
    if(*flag==0)
    for (int k = 0; k < 4; k++)
    {
        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++) {
                printf("%.4f ", result[k].Matrix[i][j]*180/PI);
            }
            printf("\n");
        }
        printf("\n");
    }
    else
        printf("gg\n");

    /*-------------------------------------------------------路径规划----------------------------------------------------*/

        
    Set_Target_point(fdresult[0].Matrix[0][3],fdresult[0].Matrix[1][3],fdresult[0].Matrix[2][3],2);//点0
    Set_Target_point(fdresult[1].Matrix[0][3],fdresult[1].Matrix[1][3],fdresult[1].Matrix[2][3],2);//1
    Set_Target_point(fdresult[2].Matrix[0][3],fdresult[2].Matrix[1][3],fdresult[2].Matrix[2][3],2);//2
    Set_Target_point(fdresult[3].Matrix[0][3],fdresult[3].Matrix[1][3],fdresult[3].Matrix[2][3],2);//3

    Joint_Space_Preprocessing();
for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
                printf("%f ",temp_d[i][j]);
        }
        printf("\n");
    }

for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
                printf("%f ",temp_d[i][j]);
        }
        printf("\n");
    }
printf("\n");
for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
                printf("%f ",temp_dd[i][j]);
        }
        printf("\n");
    }
printf("\n");
printf("\n");
printf("\n");
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<6;k++){
                printf("%f ",a_quintic[i][j][k]);
            }
            printf("\n");
        }
        printf("\n");
    }
    for(int i=0;i<400;i++)
    {
        //for(int j=0;j<4;j++)
        //{
            FunTimes+=1;
            Joint_Space_Via_Path_Planning(1,a_quintic[3][0]);
            printf("x:%f v:%f a:%f\n",Next_Angle*180/PI,Next_Omega*180/PI,Next_Acceleration*180/PI);
        //}
    }
	for(int i=0;i<200;i++)
    {

	}
    return 0;
}







