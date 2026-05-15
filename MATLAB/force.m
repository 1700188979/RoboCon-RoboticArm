% 参数定义
m1 = 1; k1 = 10; F1 = @(t) 5 * sin(t); % 第一个关节
m2 = 1; k2 = 15; F2 = @(t) 5 * cos(t); % 第二个关节
 
% 定义微分方程
% 状态变量 y = [x1; v1; x2; v2]
dynamics_arm = @(t, y) [
    y(2);
    (F1(t) - k1 * y(1)) / m1;
    y(4);
    (F2(t) - k2 * y(3)) / m2
];
 
% 初始条件
y0_arm = [0; 0; 0; 0];
 
% 时间范围
tspan_arm = [0 20];
 
% 求解微分方程
[t_arm, y_arm] = ode45(dynamics_arm, tspan_arm, y0_arm);
 
% 提取位移
x1_arm = y_arm(:,1);
x2_arm = y_arm(:,3);

% 机械臂的长度
L1_arm = 1;
L2_arm = 1;
 
% 计算关节位置
joint1_x = x1_arm;
joint1_y = zeros(size(x1_arm));
end_effector_x = joint1_x + x2_arm;
end_effector_y = zeros(size(x2_arm));
 
figure;
axis equal;
axis([-2, 2, -1, 1]);
grid on;
hold on;
title('两关节机械臂的动态演示');
xlabel('X (m)');
ylabel('Y (m)');
 
% 初始化绘图对象
h_joint1 = plot([0, joint1_x(1)], [0, joint1_y(1)], '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_end_effector = plot([joint1_x(1), end_effector_x(1)], [joint1_y(1), end_effector_y(1)], '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
 
for i = 1:length(t_arm)
    set(h_joint1, 'XData', [0, joint1_x(i)], 'YData', [0, joint1_y(i)]);
    set(h_end_effector, 'XData', [joint1_x(i), end_effector_x(i)], 'YData', [joint1_y(i), end_effector_y(i)]);
    drawnow;
    pause(0.01); % 控制动画速度
end
hold off;