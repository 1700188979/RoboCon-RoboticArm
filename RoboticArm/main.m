% 机械臂仿真主程序（模块组合示例）
clear; clc; close all;

%% 1. 初始化机械臂模型
% 初始关节角度（rad）：[J0, J1, J2, J3]
init_theta = [0, pi/6, -pi/4, 0];
arm = RoboticArmModel(init_theta);

%% 2. 正运动学计算（初始姿态）
[x0, y0, z0] = ForwardKinematics.getEndPosition(arm);
fprintf('初始末端位置：x=%.3fm, y=%.3fm, z=%.3fm\n', x0, y0, z0);

% 绘制初始姿态
ArmVisualization.plotArm(arm, '初始姿态');

%% 3. 逆运动学计算（目标位置）
% 设定目标末端位置（m）
target_x = 0.7;
target_y = 0.2;
target_z = 0.4;
fprintf('\n目标末端位置：x=%.3fm, y=%.3fm, z=%.3fm\n', target_x, target_y, target_z);

% 求解逆解
[solutions, err_flag] = InverseKinematics.calculateAllSolutions(arm, target_x, target_y, target_z);
if err_flag
    error('目标位置超出机械臂工作空间！');
end

% 选择最佳解
best_theta = InverseKinematics.chooseBestSolution(arm, solutions);
fprintf('最佳关节角度（rad）：J0=%.3f, J1=%.3f, J2=%.3f, J3=%.3f\n', ...
        best_theta(1), best_theta(2), best_theta(3), best_theta(4));

%% 4. 更新机械臂姿态并可视化
arm.updateJointAngle(best_theta);
[x_final, y_final, z_final] = ForwardKinematics.getEndPosition(arm);
fprintf('实际到达位置：x=%.3fm, y=%.3fm, z=%.3fm\n', x_final, y_final, z_final);

% 绘制目标姿态
ArmVisualization.plotArm(arm, '目标姿态');
pause(1);

%% 5. 运动动画演示（从初始姿态到目标姿态）
arm.updateJointAngle(init_theta);  % 恢复初始姿态
total_time = 3;  % 运动总时间（秒）
fprintf('\n开始运动动画（%d秒）...\n', total_time);
ArmVisualization.animateMovement(arm, best_theta, total_time);

fprintf('\n仿真结束！\n');