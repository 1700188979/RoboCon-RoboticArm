#include <math.h>  // 需包含数学库（cos、sin等函数）

// 定义4x4矩阵类型（float型）
typedef float Matrix4x4[4][4];

// 计算T04矩阵，结果存入result（float类型）
void calc_T04(float t1, float t2, float t3, float t4, 
              float a0, float a1, float a2, float a3, 
              Matrix4x4 result) {
    // 合并角度（弧度）
    float t23 = t2 + t3;       // t2 + t3
    float t234 = t2 + t3 + t4; // t2 + t3 + t4
    
    // 三角函数计算（缩写：c=cos, s=sin；数字=角度组合）
    float c1 = cosf(t1);    float s1 = sinf(t1);
    float c2 = cosf(t2);    float s2 = sinf(t2);
    float c23 = cosf(t23);  float s23 = sinf(t23);
    float c234 = cosf(t234);float s234 = sinf(t234);
    
    // 计算旋转矩阵部分（3x3）
    result[0][0] = c1 * c234;       // R11
    result[0][1] = -c1 * s234;      // R12
    result[0][2] = s1;              // R13
    
    result[1][0] = s1 * c234;       // R21
    result[1][1] = -s1 * s234;      // R22
    result[1][2] = -c1;             // R23
    
    result[2][0] = s234;            // R31
    result[2][1] = c234;            // R32
    result[2][2] = 0.0f;            // R33
    
    // 计算平移向量部分（3x1）
    result[0][3] = a0*c1 + a1*c1*c2 + a2*c1*c23 + a3*c1*c234;  // Px
    result[1][3] = a0*s1 + a1*s1*c2 + a2*s1*c23 + a3*s1*c234;  // Py
    result[2][3] = a1*s2 + a2*s23 + a3*s234;                   // Pz
    
    // 第四行固定值
    result[3][0] = 0.0f;
    result[3][1] = 0.0f;
    result[3][2] = 0.0f;
    result[3][3] = 1.0f;
}

#include <stdio.h>
float Angle_Normalization(float theta)
{
	if (fabs(theta)<1e-3)	return 0;
	if (theta>M_PI)			return theta-2*M_PI;
	if (theta<-M_PI)			return theta+2*M_PI;
	return theta;
}
int main() {
    // 输入参数（t2=90°=pi/2弧度）
    float t1 = 45.0f*M_PI/180;
    float t2 = 39.0f*M_PI/180;  // 需定义M_PI（部分编译器需#define _USE_MATH_DEFINES）
    float t3 = -51.52f*M_PI/180;
    float t4 = 0.0f*M_PI/180;
    t4=Angle_Normalization(-M_PI/2-t2-t3);
    // t4=179.49*M_PI/180;
    float a0 = 0.0f, a1 = 0.39f, a2 = 0.36f, a3 = 0.14f;
    
    // 定义result存储矩阵（float类型）
    Matrix4x4 result;
    
    // 计算T04
    calc_T04(t1, t2, t3, t4, a0, a1, a2, a3, result);
    
    // 打印结果（验证）
    printf("T04矩阵（float存储）：\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%8.5f ", result[i][j]);
        }
        printf("\n");
    }
    return 0;
}