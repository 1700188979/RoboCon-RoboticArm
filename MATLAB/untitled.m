% -------------------------- 4轴机械臂逆解验证脚本（无工具箱依赖，修正变量作用域） --------------------------
% 功能：输入XYZ坐标，输出θ₁~θ₄（弧度+角度），并验证正解是否匹配目标位置
% 匹配你的参数：a2=0.39, a3=0.36, a4=0.14；末端竖直向下约束（s234=-1）

clear; clc; close all;

% ====================== 第一步：输入你的目标XYZ坐标（替换为你要验证的数值） ======================
x_target = 0.21205;  % 示例：θ₁=45°时的理论x
y_target = 0.21205;  % 示例：θ₁=45°时的理论y（y=x，因θ₁=45°）
z_target = 0.46540;  % 示例：目标z坐标
fprintf('目标位置：X=%.4fm, Y=%.4fm, Z=%.4fm\n\n', x_target, y_target, z_target);

% ====================== 第二步：定义连杆参数（全局统一，避免作用域问题） ======================
a2 = 0.39;  % 关节1-2长度（与你的C代码一致）
a3 = 0.36;  % 关节2-3长度（与你的C代码一致）
a4 = 0.14;  % 关节3-末端长度（与你的C代码一致）

% ====================== 第三步：调用逆解函数求解关节角 ======================
[q_sol, success] = arm_inverse_kinematics(x_target, y_target, z_target, a2, a3, a4);

% ====================== 第四步：输出结果并验证 ======================
if success
    % 转换为角度（方便对比）
    q_deg = rad2deg(q_sol);
    
    fprintf('逆解成功！关节角结果：\n');
    fprintf('θ₁ = %.2f° (%.4f rad)\n', q_deg(1), q_sol(1));
    fprintf('θ₂ = %.2f° (%.4f rad)\n', q_deg(2), q_sol(2));
    fprintf('θ₃ = %.2f° (%.4f rad)\n', q_deg(3), q_sol(3));
    fprintf('θ₄ = %.2f° (%.4f rad)\n\n', q_deg(4), q_sol(4));
    
    % 验证：将关节角代入正解，检查是否与目标位置一致
    [x_calc, y_calc, z_calc] = arm_forward_kinematics(q_sol(1), q_sol(2), q_sol(3), q_sol(4), a2, a3, a4);
    fprintf('正解验证：\n');
    fprintf('计算位置：X=%.4fm, Y=%.4fm, Z=%.4fm\n', x_calc, y_calc, z_calc);
    fprintf('位置误差：ΔX=%.6fm, ΔY=%.6fm, ΔZ=%.6fm\n', ...
        x_calc-x_target, y_calc-y_target, z_calc-z_target);
    
    % 验证末端竖直向下约束（s234=sin(θ₂+θ₃+θ₄)应≈-1）
    s234 = sin(q_sol(2) + q_sol(3) + q_sol(4));
    fprintf('末端竖直约束验证：sin(θ₂+θ₃+θ₄)=%.6f（理论值=-1）\n', s234);
else
    fprintf('逆解失败！目标位置超出工作空间或参数错误。\n');
end

% -------------------------- 逆解核心函数（复现你的C代码逻辑，显式传入参数） --------------------------
function [q_sol, success] = arm_inverse_kinematics(x, y, z, a2, a3, a4)
    pi_val = pi;
    success = true;
    
    % 1. 求解θ₁（两组解：θ₁和θ₁+π，此处取第一组解，可切换对比）
    theta1 = atan2(y, x);
    theta1 = angle_normalize(theta1, pi_val);  % 归一化到(-π, π]
    
    % 2. 计算K1和K2（水平/垂直约束）
    cos_theta1 = cos(theta1);
    if abs(cos_theta1) < 1e-6  % 避免除零
        cos_theta1 = 1e-6;
    end
    K1 = x / cos_theta1;  % 你的C代码中a1=0，故K1=x/cosθ₁ - a1 = x/cosθ₁
    K2 = a4 + z;          % 垂直约束：K2=末端Z偏移+a4
    
    % 3. 求解θ₃（核心：基于K1²+K2²推导）
    C = (K1^2 + K2^2 - a2^2 - a3^2) / (2*a2*a3);
    C = max(min(C, 1), -1);  % 裁剪到[-1,1]，避免acos参数异常
    theta3 = acos(C);        % 第一组解（另一组解为2π-theta3，可修改为 theta3=2*pi_val-theta3 切换）
    theta3 = angle_normalize(theta3, pi_val);
    
    % 4. 求解θ₂（基于θ₃，矩阵求逆+atan2，无象限歧义）
    A = -a3 * sin(theta3);
    B = a2 + a3 * cos(theta3);
    denom = A^2 + B^2;
    if denom < 1e-6  % 避免除零
        success = false;
        q_sol = [0,0,0,0];
        return;
    end
    c2 = (B*K1 - A*K2) / denom;
    s2 = (A*K1 + B*K2) / denom;
    % 归一化修正（确保c2²+s2²=1）
    norm_val = sqrt(c2^2 + s2^2);
    if norm_val > 1e-6
        c2 = c2 / norm_val;
        s2 = s2 / norm_val;
    else
        c2 = 1;
        s2 = 0;
    end
    theta2 = atan2(s2, c2);
    theta2 = angle_normalize(theta2, pi_val);
    
    % 5. 求解θ₄（满足末端竖直向下：θ₂+θ₃+θ₄=3π/2）
    theta4 = 3*pi_val/2 - theta2 - theta3;
    theta4 = angle_normalize(theta4, pi_val);
    
    % 输出最终解（θ₁, θ₂, θ₃, θ₄）
    q_sol = [theta1, theta2, theta3, theta4];
end

% -------------------------- 正解函数（匹配你的C代码正解公式，显式传入参数） --------------------------
function [x, y, z] = arm_forward_kinematics(theta1, theta2, theta3, theta4, a2, a3, a4)
    c1 = cos(theta1);
    s1 = sin(theta1);
    c2 = cos(theta2);
    s2 = sin(theta2);
    c23 = cos(theta2 + theta3);
    s23 = sin(theta2 + theta3);
    c234 = cos(theta2 + theta3 + theta4);
    s234 = sin(theta2 + theta3 + theta4);
    
    % 完全匹配你的正解公式
    x = c1 * (a2*c2 + a3*c23 + a4*c234);
    y = s1 * (a2*c2 + a3*c23 + a4*c234);
    z = a2*s2 + a3*s23 + a4*s234;
end

% -------------------------- 辅助函数：角度归一化到(-π, π] --------------------------
function theta_norm = angle_normalize(theta, pi_val)
    if abs(theta) < 1e-3
        theta_norm = 0;
        return;
    end
    while theta > pi_val
        theta = theta - 2*pi_val;
    end
    while theta <= -pi_val
        theta = theta + 2*pi_val;
    end
    theta_norm = theta;
end