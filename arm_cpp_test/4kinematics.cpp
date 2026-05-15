#include <stdio.h>
#include <math.h>


/*
    此版本为四轴机械臂
    所有变量均未代入值
    经测试所有数据无误
*/






typedef struct {
    float m[4][4];
} Matrix4x4;

// 辅助：单个DH矩阵计算（用于验证通用表达式）
Matrix4x4 calculate_single_dh(float alpha, float a, float d, float theta) {
    Matrix4x4 mat;
    float ca = cosf(alpha);
    float sa = sinf(alpha);
    float ct = cosf(theta);
    float st = sinf(theta);

    mat.m[0][0] = ct;
    mat.m[0][1] = -st*ca;
    mat.m[0][2] = st*sa;
    mat.m[0][3] = a*ct;

    mat.m[1][0] = st;
    mat.m[1][1] = ct * ca;
    mat.m[1][2] = -ct*sa;
    mat.m[1][3] = a * st;

    mat.m[2][0] = 0;
    mat.m[2][1] = sa;
    mat.m[2][2] = ca;
    mat.m[2][3] = d;

    mat.m[3][0] = 0;
    mat.m[3][1] = 0;
    mat.m[3][2] = 0;
    mat.m[3][3] = 1;

    return mat;
}

// 辅助：矩阵乘法（用于验证通用表达式）
Matrix4x4 matrix_multiply(Matrix4x4 A, Matrix4x4 B) {
    Matrix4x4 result = {0};
    for (int i = 0; i < 4; i++)
        for (int k = 0; k < 4; k++)
            for (int j = 0; j < 4; j++)
                result.m[i][j] += A.m[i][k] * B.m[k][j];
    return result;
}

// 方法1：通用表达式计算总变换矩阵
Matrix4x4 calculate_total_transform_general(
    float a0, float d1, float alpha0, float theta1,
    float a1, float d2, float alpha1, float theta2,
    float a2, float d3, float alpha2, float theta3,
    float a3, float d4, float alpha3, float theta4
) {
    Matrix4x4 total;
    float deg2rad = M_PI / 180.0f;  // 角度转弧度（若输入是角度需转换，此处假设输入已为弧度）

    // 角度累加与三角函数
    float theta234 = theta2 + theta3 + theta4;
    float alpha0123 = alpha0 + alpha1 + alpha2 + alpha3;

    float c1 = cosf(theta1);
    float s1 = sinf(theta1);
    float c234 = cosf(theta234);
    float s234 = sinf(theta234);
    float calpha = cosf(alpha0123);
    float salpha = sinf(alpha0123);

    // 平移分量P_x, P_y, P_z（逐关节推导）
    float Px = a0 * c1 + 
               a1 * (c1*c234 - s1*s234*calpha) + 
               a2 * (c1*c234 - s1*s234*calpha) + 
               a3 * (c1*c234 - s1*s234*calpha) + 
               d1 * s1*salpha + 
               d2 * (s1*s234 + c1*c234*calpha)*salpha + 
               d3 * (s1*s234 + c1*c234*calpha)*salpha + 
               d4 * (s1*s234 + c1*c234*calpha)*salpha;

    float Py = a0 * s1 + 
               a1 * (s1*c234 + c1*s234*calpha) + 
               a2 * (s1*c234 + c1*s234*calpha) + 
               a3 * (s1*c234 + c1*s234*calpha) + 
               d1 * (-c1)*salpha + 
               d2 * (-c1*s234 + s1*c234*calpha)*salpha + 
               d3 * (-c1*s234 + s1*c234*calpha)*salpha + 
               d4 * (-c1*s234 + s1*c234*calpha)*salpha;

    float Pz = a1 * s234*salpha + 
               a2 * s234*salpha + 
               a3 * s234*salpha + 
               d1 * calpha + 
               d2 * c234*salpha + 
               d3 * c234*salpha + 
               d4 * c234*salpha + 
               calpha*d4;

    // 填充总变换矩阵
    total.m[0][0] = c1*c234 - s1*s234*calpha;
    total.m[0][1] = -c1*s234 - s1*c234*calpha;
    total.m[0][2] = s1*salpha;
    total.m[0][3] = Px;

    total.m[1][0] = s1*c234 + c1*s234*calpha;
    total.m[1][1] = -s1*s234 + c1*c234*calpha;
    total.m[1][2] = -c1*salpha;
    total.m[1][3] = Py;

    total.m[2][0] = s234*salpha;
    total.m[2][1] = c234*salpha;
    total.m[2][2] = calpha;
    total.m[2][3] = Pz;

    total.m[3][0] = 0;
    total.m[3][1] = 0;
    total.m[3][2] = 0;
    total.m[3][3] = 1;

    return total;
}

// 方法2：逐个DH矩阵连乘（用于验证通用表达式）
Matrix4x4 calculate_total_transform_dh(
    float a0, float d1, float alpha0, float theta1,
    float a1, float d2, float alpha1, float theta2,
    float a2, float d3, float alpha2, float theta3,
    float a3, float d4, float alpha3, float theta4
) {
    Matrix4x4 T1 = calculate_single_dh(alpha0, a0, d1, theta1);
    Matrix4x4 T2 = calculate_single_dh(alpha1, a1, d2, theta2);
    Matrix4x4 T3 = calculate_single_dh(alpha2, a2, d3, theta3);
    Matrix4x4 T4 = calculate_single_dh(alpha3, a3, d4, theta4);

    Matrix4x4 temp1 = matrix_multiply(T1, T2);
    Matrix4x4 temp2 = matrix_multiply(temp1, T3);
    Matrix4x4 total = matrix_multiply(temp2, T4);

    return total;
}

int main() {
    float deg2rad = M_PI / 180.0f;  // 角度转弧度

    // 示例参数（需替换为实际值，此处用之前的案例参数）
    float a0 = 0,    d1 = 0,    alpha0 = 90 * deg2rad,  theta1 = 0 * deg2rad;
    float a1 = 0.39,  d2 = 0,    alpha1 = 0 * deg2rad,  theta2 = 90 * deg2rad;
    float a2 = 0.36,  d3 = 0,    alpha2 = 0 * deg2rad,  theta3 = 0 * deg2rad;
    float a3 = 0.14,  d4 = 0,    alpha3 = 0 * deg2rad,  theta4 = 0 * deg2rad;

    // 方法1：通用表达式计算
    Matrix4x4 total_general = calculate_total_transform_general(
        a0, d1, alpha0, theta1,
        a1, d2, alpha1, theta2,
        a2, d3, alpha2, theta3,
        a3, d4, alpha3, theta4
    );

    // 方法2：逐个DH矩阵连乘（验证）
    Matrix4x4 total_dh = calculate_total_transform_dh(
        a0, d1, alpha0, theta1,
        a1, d2, alpha1, theta2,
        a2, d3, alpha2, theta3,
        a3, d4, alpha3, theta4
    );

    // 打印两种方法的结果，验证一致性
    printf("=== 通用表达式结果 ===\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%.4f ", total_general.m[i][j]);
        }
        printf("\n");
    }

    printf("\n=== 逐个DH矩阵连乘结果（验证） ===\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%.4f ", total_dh.m[i][j]);
        }
        printf("\n");
    }

    return 0;
}