classdef ArmVisualization
    methods (Static)
        % 绘制机械臂三维姿态
        function plotArm(arm_model, varargin)
            % varargin(1)：可选标题，varargin(2)：是否保持之前的图
            title_str = '机械臂仿真姿态';
            hold_flag = false;
            if nargin >= 2
                title_str = varargin{1};
            end
            if nargin >= 3
                hold_flag = varargin{2};
            end
            
            % 计算各关节位置
            [joints_x, joints_y, joints_z] = ArmVisualization.calculateJointPositions(arm_model);
            
            % 创建/激活图形窗口
            if ~hold_flag
                figure('Name', '机械臂仿真', 'Position', [100, 100, 800, 600]);
            end
            hold on; grid on; axis equal;
            
            % 绘制机械臂连杆（实线）和关节（圆点）
            plot3(joints_x, joints_y, joints_z, ...
                  'LineWidth', 3, 'Color', [0.2, 0.6, 0.8], ...
                  'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.8, 0.2, 0.3]);
            
            % 标注关节
            for i = 1:length(joints_x)
                text(joints_x(i)+0.02, joints_y(i)+0.02, joints_z(i)+0.02, ...
                     sprintf('J%d', i-1), 'FontSize', 10);
            end
            
            % 坐标轴设置
            xlabel('X轴 (m)', 'FontSize', 12);
            ylabel('Y轴 (m)', 'FontSize', 12);
            zlabel('Z轴 (m)', 'FontSize', 12);
            title(title_str, 'FontSize', 14);
            view(45, 30);  % 视角调整
            hold off;
        end
        
        % 计算所有关节的三维坐标（包括基座J0和末端J4）
        function [joints_x, joints_y, joints_z] = calculateJointPositions(arm_model)
            dh = arm_model.getDHParam();
            joint_num = arm_model.joint_num;
            T_total = eye(4);
            joints = zeros(4, joint_num + 1);  % 4x5（齐次坐标）
            joints(:,1) = [0; 0; 0; 1];  % J0（基座）坐标
            
            % 累积变换矩阵，计算每个关节的位置
            for i = 1:joint_num
                T_i = ForwardKinematics.getDHTransform( ...
                    dh.alpha(i), dh.a(i), dh.d(i), dh.theta(i) ...
                );
                T_total = T_total * T_i;
                joints(:, i+1) = T_total * [0; 0; 0; 1];  % 关节中心坐标
            end
            
            % 提取x,y,z坐标
            joints_x = joints(1, :);
            joints_y = joints(2, :);
            joints_z = joints(3, :);
        end
        
        % 动画演示（机械臂从初始姿态运动到目标姿态）
        function animateMovement(arm_model, target_theta, total_time)
            current_theta = arm_model.getCurrentJointAngle();
            dt = 0.01;  % 时间步长
            t = 0:dt:total_time;
            num_steps = length(t);
            
            % 五次多项式轨迹插值（平滑运动）
            for j = 1:arm_model.joint_num
                theta_t(:,j) = TrajectoryPlanning.quinticPolynomial( ...
                    t, total_time, current_theta(j), 0, 0, target_theta(j), 0, 0 ...
                );
            end
            
            % 逐帧绘制
            figure('Name', '机械臂运动动画', 'Position', [200, 200, 800, 600]);
            for i = 1:num_steps
                arm_model.updateJointAngle(theta_t(i,:));
                ArmVisualization.plotArm(arm_model, sprintf('运动时间: %.2fs', t(i)), true);
                drawnow;  % 强制刷新画面
                pause(dt);
            end
        end
    end
end
