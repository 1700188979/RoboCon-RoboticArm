clc; clear; close all;

%% 1. 机械臂建模（官方SerialLink库）
% D-H参数表：[theta, d, a, alpha]，默认转动关节
L1 = Link([0, 0, 0, pi/2], 'standard');
L2 = Link([0, 0, 0.39, 0], 'standard');
L3 = Link([0, 0, 0.36, 0], 'standard');
L4 = Link([0, 0, 0.14, 0], 'standard');

% 组建机械臂（官方库核心类）
robot = SerialLink([L1, L2, L3, L4], 'name', '4DOF Arm');
robot.display();  % 显示机械臂参数（官方库自带方法）

%% 2. 关节空间轨迹规划（官方jtraj函数）
init_q = [0, 0, 0, 0];       % 初始关节角
targ_q = [0, -pi/6, -pi/5, pi/6];  % 目标关节角
step_num = 200;              % 轨迹步数

% 官方函数生成关节空间轨迹（角度、速度、加速度）
[q, qd, qdd] = jtraj(init_q, targ_q, step_num);

%% 3. 核心可视化（官方plot/teach函数）
figure('Name', '关节空间仿真');
subplot(2,2,1:2);
% 3D显示机械臂运动轨迹（官方plot函数动态驱动）
robot.plot(q, 'workspace', [-1, 1, -1, 1, -1, 1]);
title('关节空间机械臂运动仿真');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3); grid on;

% 关节角度曲线（简化显示）
subplot(2,2,3);
plot(q, 'LineWidth', 1.2);
xlabel('步数'); ylabel('关节角度 (rad)');
title('关节角度轨迹');
grid on; legend({'关节1','关节2','关节3','关节4'}, 'Location','best');

% 关节速度曲线（简化显示）
subplot(2,2,4);
plot(qd, 'LineWidth', 1.2);
xlabel('步数'); ylabel('关节速度 (rad/s)');
title('关节速度轨迹');
grid on; legend({'关节1','关节2','关节3','关节4'}, 'Location','best');
