#include <stdio.h>
#include <math.h>

typedef struct {
    float Matrix[4][4];
} Matrix4x4;

Matrix4x4 Calculate_kinematics_Transform(
    float a0, float d1, float alpha0, float theta1,  // 关节1参数
    float a1, float d2, float alpha1, float theta2,  // 关节2参数
    float a2, float d3, float alpha2, float theta3,  // 关节3参数
    float a3, float d4, float alpha3, float theta4   // 关节4参数
)
{
    Matrix4x4 total;
	
    // 角度累加与三角函数
    float theta234 = theta2 + theta3 + theta4;

    float c1 = cosf(theta1);
    float s1 = sinf(theta1);
    float c234 = cosf(theta234);
    float s234 = sinf(theta234);

    // 填充总变换矩阵
    total.Matrix[0][0] = c1*c234;
    total.Matrix[0][1] = -c1*s234;
    total.Matrix[0][2] = -s1;
    total.Matrix[0][3] = c1*c234*(a1+a2+a3);

    total.Matrix[1][0] = s1*c234;
    total.Matrix[1][1] = -s1*s234;
    total.Matrix[1][2] = c1;
    total.Matrix[1][3] = s1*c234*(a1+a2+a3);

    total.Matrix[2][0] = -s234;
    total.Matrix[2][1] = -c234;
    total.Matrix[2][2] = 0;
    total.Matrix[2][3] = -s234*(a1+a2+a3);

    total.Matrix[3][0] = 0;
    total.Matrix[3][1] = 0;
    total.Matrix[3][2] = 0;
    total.Matrix[3][3] = 1;

    return total;
}
int main() {
    // 示例：假设所有关节的α=0，a=0，d=0（仅旋转关节）
    float PI=3.1415926;
    float a0=0, d1=0, alpha0=0, theta1=PI/4;
    float a1=0.5, d2=0, alpha1=-PI/2, theta2=80;
    float a2=0.2, d3=0, alpha2=0, theta3=-30;
    float a3=0.5, d4=0, alpha3=0, theta4=-70;

     a0=0, d1=0, alpha0=0, theta1=0;
     a1=0.39, d2=0, alpha1=-PI/2, theta2=-90*PI/180;
     a2=0.36, d3=0, alpha2=0, theta3=90*PI/180;
     a3=0.14, d4=0, alpha3=0, theta4=45*PI/180;

    Matrix4x4 total = Calculate_kinematics_Transform(
        a0, d1, alpha0, theta1,
        a1, d2, alpha1, theta2,
        a2, d3, alpha2, theta3,
        a3, d4, alpha3, theta4
    );

    // 打印矩阵验证（此时应为单位矩阵）
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            printf("%.4f ", total.Matrix[i][j]);
        }
        printf("\n");
    }

    return 0;
}
