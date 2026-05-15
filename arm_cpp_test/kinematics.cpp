#include<stdio.h>
#include <math.h>

// 定义4x4矩阵结构体
typedef struct {
    float m[4][4];
} Matrix4x4;

// 计算总变换矩阵（基于通用表达式）
Matrix4x4 calculate_total_transform(
    float a0, float d1, float alpha0, float theta1,  // 关节1参数
    float a1, float d2, float alpha1, float theta2,  // 关节2参数
    float a2, float d3, float alpha2, float theta3,  // 关节3参数
    float a3, float d4, float alpha3, float theta4,  // 关节4参数
    float a4, float d5, float alpha4, float theta5   // 关节5参数
) {
    Matrix4x4 total;

    // 计算角度累加与三角函数
    float theta2345 = theta2 + theta3 + theta4 + theta5;
    float alpha01234 = alpha0 + alpha1 + alpha2 + alpha3 + alpha4;

    float c1 = cosf(theta1);
    float s1 = sinf(theta1);
    float c2345 = cosf(theta2345);
    float s2345 = sinf(theta2345);
    float calpha = cosf(alpha01234);
    float salpha = sinf(alpha01234);

    // 计算平移分量P_x, P_y, P_z
    // 平移由各关节的a_{i-1}和d_i经旋转后累加，需逐关节展开
    float Px = a0 * c1 + a1 * (c1*c2345 - s1*s2345*calpha) + 
               a2 * (c1*c2345 - s1*s2345*calpha) +  // 示例，需根据实际参数修正
               a3 * (c1*c2345 - s1*s2345*calpha) + 
               a4 * (c1*c2345 - s1*s2345*calpha) + 
               d1 * s1*salpha + d2 * (s1*s2345 + c1*c2345*calpha)*salpha +  // 偏距d的贡献
               d3 * (s1*s2345 + c1*c2345*calpha)*salpha + 
               d4 * (s1*s2345 + c1*c2345*calpha)*salpha + 
               d5 * (s1*s2345 + c1*c2345*calpha)*salpha;

    float Py = a0 * s1 + a1 * (s1*c2345 + c1*s2345*calpha) + 
               a2 * (s1*c2345 + c1*s2345*calpha) + 
               a3 * (s1*c2345 + c1*s2345*calpha) + 
               a4 * (s1*c2345 + c1*s2345*calpha) + 
               d1 * (-c1)*salpha + d2 * (-c1*s2345 + s1*c2345*calpha)*salpha + 
               d3 * (-c1*s2345 + s1*c2345*calpha)*salpha + 
               d4 * (-c1*s2345 + s1*c2345*calpha)*salpha + 
               d5 * (-c1*s2345 + s1*c2345*calpha)*salpha;

    float Pz = a0 * 0 + a1 * s2345*salpha + a2 * s2345*salpha + 
               a3 * s2345*salpha + a4 * s2345*salpha + 
               d1 * calpha + d2 * c2345*salpha + d3 * c2345*salpha + 
               d4 * c2345*salpha + d5 * c2345*salpha + calpha*d5;  // 偏距d的z向贡献

    // 按通用表达式填充旋转与平移部分
    total.m[0][0] = c1*c2345 - s1*s2345*calpha;
    total.m[0][1] = -c1*s2345 - s1*c2345*calpha;
    total.m[0][2] = s1*salpha;
    total.m[0][3] = Px;

    total.m[1][0] = s1*c2345 + c1*s2345*calpha;
    total.m[1][1] = -s1*s2345 + c1*c2345*calpha;
    total.m[1][2] = -c1*salpha;
    total.m[1][3] = Py;

    total.m[2][0] = s2345*salpha;
    total.m[2][1] = c2345*salpha;
    total.m[2][2] = calpha;
    total.m[2][3] = Pz;

    total.m[3][0] = 0;
    total.m[3][1] = 0;
    total.m[3][2] = 0;
    total.m[3][3] = 1;

    return total;
}

int main() {
    // 示例：假设所有关节的α=0，a=0，d=0（仅旋转关节）
    float PI=3.1415926;
    float a0=0, d1=0, alpha0=0, theta1=PI/4;
    float a1=0.5, d2=0, alpha1=-PI/2, theta2=80;
    float a2=0.2, d3=0, alpha2=0, theta3=-30;
    float a3=0.5, d4=0, alpha3=0, theta4=-70;
    float a4=0.2, d5=0, alpha4=0, theta5=-30;

     a0=0, d1=0, alpha0=0, theta1=0;
     a1=0.5, d2=0, alpha1=-PI/2, theta2=80*PI/180;
     a2=0.2, d3=0, alpha2=0, theta3=-30*PI/180;
     a3=0.5, d4=0, alpha3=0, theta4=-70*PI/180;
     a4=0.2, d5=0, alpha4=0, theta5=-30*PI/180;

     a0=0, d1=0, alpha0=0, theta1=0;
     a1=0.5, d2=0, alpha1=-PI/2, theta2=-90*PI/180;
     a2=0.2, d3=0, alpha2=0, theta3=0;
     a3=0.5, d4=0, alpha3=0, theta4=0;
     a4=0.2, d5=0, alpha4=0, theta5=0;

    Matrix4x4 total = calculate_total_transform(
        a0, d1, alpha0, theta1,
        a1, d2, alpha1, theta2,
        a2, d3, alpha2, theta3,
        a3, d4, alpha3, theta4,
        a4, d5, alpha4, theta5
    );

    // 打印矩阵验证（此时应为单位矩阵）
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            printf("%.4f ", total.m[i][j]);
        }
        printf("\n");
    }

    return 0;
}
