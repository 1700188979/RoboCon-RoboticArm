//
// Created by Lenovo on 25-11-14.
//

#ifndef ROBOTIC_ARM_H
#define ROBOTIC_ARM_H

#define Shortest_Interval 0.001f                 //两点之间最小时间间隔5ms
#define POS_MAX_NUM 50                          //路径规划中间点+终点最多20个点
#define Turning_Acceleration 0.5f               //转向时的角加速度绝对值0.5m/(s*s)

// 定义4x4矩阵结构体
typedef struct {
    float Matrix[4][4];
} Matrix4x4;

enum Enum_Order_OF_Robotic_Arm_Path_Planning_Curve
{
    Third_Order = 0,
    Fifth_Order = 1
};

enum Pose_Orientation
{
    Facing_Forward = 0,
    Facing_Downward = 1
};

enum Motor_Rotation_Adjust
{
    Forward_Rotation = 0,
    Reverse_Rotation = 1
};

/**
 * @brief 关节DH参数
 * 沿 Z平移di
 * 沿 X平移ai_1
 * 绕 X旋转alphai_1
 * 旋转角度Angle
 */
typedef struct Transformation_Matrix_Para
{
    float alpha;
    float a;
    float d;
    float Now_Angle;
    float Now_Omega;
    float Now_Acceleration;
    float Next_Angle;
    float Next_Omega;
    float Next_Acceleration;
    float bias;             //零点偏移 实际-理论    设置值=目标点-bias
    float MAX_Angle;
    float MIN_Angle;
    float Reduction_Ratio;
    Matrix4x4 Matrix;
    Motor_Rotation_Adjust Rotation;
}Transformation_Matrix_Para;

/**
 * @brief 机械臂类
 *
 */
class Class_Robotic_arm
{
public:

    void Init();

    void Init_Maxtrix();

    void robot_cal_T(Transformation_Matrix_Para *TrMatrix);

    inline Matrix4x4 robot_cal_4T(Matrix4x4 T1,Matrix4x4 T2,Matrix4x4 T3,Matrix4x4 T4);

    Matrix4x4 Calculate_kinematics_Transform();

    int Kinematics_Inverse_Preprocessing(float x, float y, float z);

    Matrix4x4 Calculate_kinematics_Inverse(float x, float y, float z, int* err_flag, Pose_Orientation Attitude);

    float Cal_theta2(float theta3_11,float K1,float K2,float a2,float a3);

    float Cal_theta3(float K1,float K2,float a2,float a3,int* err_flag);

    void Choose_Best_Angle_Group_Processing(Matrix4x4 Angle_Group, int* err_num, int err_group_flag[4]);

    int Choose_Best_Angle_Group(Matrix4x4 Angle_Group, float Last_Best_Group[4], int* err_flag);

    void Joint_Space_Quintic_Cal_Via_Para(float T,float theta0, float d_theta0, float dd_theta0,float thetaf, float d_thetaf, float dd_thetaf, float a[6], Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Target_Order_Num);

    void Joint_Space_Automatic_Para_Generation(int best_flag[]);

    int Joint_Space_Preprocessing();

    void Joint_Space_Via_Path_Planning(int Ordinal_Num, float a[6]);

    void Joint_Space_4Angle_Group_Path_Planning();

    void Set_Target_point(float x, float y, float z, float t, Pose_Orientation Attitude, Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Target_Order_Num);

    void Clear_Path_Planning();

    void Robotic_Motor_Get();

    void Robotic_Motor_Set();

    void Robotic_TIM_Send_PeriodElapsedCallback();

    int Joint_Space_Dynamic_Tuning();

    int path_finish_flag=0;

    float FunTimes = 0;

    Transformation_Matrix_Para DH_arm_motor[4]; //电机数据及其对应连轴的DH参数

    Matrix4x4 Error_Result;                     //Error_Result为空，作为错误反馈

    int acc_num=0, now_num=0;               //总点数,当前点数

    float position[POS_MAX_NUM][5];             //4位分别代表x,y,z,T,N,dir，第一个目标点存储在0位，N=T/Shortest_Interval

    float a_quintic[POS_MAX_NUM][4][6];         //4位分别代表x,y,z,T，第一个目标点存储在0位

    Pose_Orientation Attitude[POS_MAX_NUM]={Facing_Downward};//默认朝下

    Enum_Order_OF_Robotic_Arm_Path_Planning_Curve Order_Num[POS_MAX_NUM]={Third_Order};//默认三阶

    Matrix4x4 Target_Angle_Matrix[POS_MAX_NUM]={0};

    float temp_d[POS_MAX_NUM][4]={{0}},temp_dd[POS_MAX_NUM][4]={{0}};

	int best_flag[POS_MAX_NUM];


protected:

};

Matrix4x4 matrix_multiply(Matrix4x4 T1, Matrix4x4 T2);
inline float Angle_Normalization(float theta);
#endif //ROBOTIC_ARM_H

