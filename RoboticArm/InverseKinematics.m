classdef InverseKinematics
    methods (Static)
        % 角度归一化（-pi ~ pi）
        function theta_norm = normalizeAngle(theta)
            theta_norm = mod(theta + pi, 2*pi) - pi;
        end
        
        % 求解theta3（肘部角度）
        function [theta3, err_flag] = calculateTheta3(K1, K2, a2, a3)
            err_flag = 0;
            C = (K1^2 + K2^2 - a2^2 - a3^2) / (2*a2*a3);
            
            % 边界判断（超出工作空间）
            if C > 1 + 1e-3 || C < -1 - 1e-3
                err_flag = 1;
                theta3 = 0;
                return;
            end
            
            % 数值稳定性处理
            C = max(min(C, 1), -1);
            theta3 = acos(C);
        end
        
        % 求解theta2（肩部角度）
        function theta2 = calculateTheta2(theta3, K1, K2, a2, a3)
            A = -a3 * sin(theta3);
            B = a2 + a3 * cos(theta3);
            
            if abs(A) < 1e-6 && abs(B) < 1e-6
                theta2 = 0;
                return;
            end
            
            % 计算theta2的正切值
            theta2 = atan2(K2 - B*(K1/A), -A - B^2/A);
        end
        
        % 逆运动学求解：输入目标位置(x,y,z)和模型，输出四组解
        function [solutions, err_flag] = calculateAllSolutions(arm_model, x, y, z)
            err_flag = 0;
            dh = arm_model.getDHParam();
            a1 = dh.a(1); a2 = dh.a(2); a3 = dh.a(3); a4 = dh.a(4);
            solutions = zeros(4, 4);  % 4组解 x 4个关节
            
            % 1. 求解theta1（腰部旋转）
            theta1_1 = atan2(y, x);
            theta1_2 = InverseKinematics.normalizeAngle(theta1_1 + pi);
            
            % 2. 计算中间参数K1, K2
            if abs(cos(theta1_1)) < 1e-6 || abs(cos(theta1_2)) < 1e-6
                err_flag = 1;
                solutions = [];
                return;
            end
            
            K1_1 = x / cos(theta1_1) - a1;
            K2_1 = z + a4;  % 末端执行器长度补偿
            K1_2 = x / cos(theta1_2) - a1;
            K2_2 = z + a4;
            
            % 3. 求解theta3（两组解：肘部向上/向下）
            [theta3_11, err1] = InverseKinematics.calculateTheta3(K1_1, K2_1, a2, a3);
            [theta3_21, err2] = InverseKinematics.calculateTheta3(K1_2, K2_2, a2, a3);
            if err1 || err2
                err_flag = 1;
                solutions = [];
                return;
            end
            theta3_12 = -theta3_11;
            theta3_22 = -theta3_21;
            
            % 4. 求解theta2
            theta2_11 = InverseKinematics.calculateTheta2(theta3_11, K1_1, K2_1, a2, a3);
            theta2_12 = InverseKinematics.calculateTheta2(theta3_12, K1_1, K2_1, a2, a3);
            theta2_21 = InverseKinematics.calculateTheta2(theta3_21, K2_2, K2_2, a2, a3);
            theta2_22 = InverseKinematics.calculateTheta2(theta3_22, K2_2, K2_2, a2, a3);
            
            % 5. 求解theta4（腕部角度）
            theta4_11 = InverseKinematics.normalizeAngle(-pi/2 - theta2_11 - theta3_11);
            theta4_12 = InverseKinematics.normalizeAngle(-pi/2 - theta2_12 - theta3_12);
            theta4_21 = InverseKinematics.normalizeAngle(-pi/2 - theta2_21 - theta3_21);
            theta4_22 = InverseKinematics.normalizeAngle(-pi/2 - theta2_22 - theta3_22);
            
            % 整理四组解
            solutions(1,:) = [theta1_1, theta2_11, theta3_11, theta4_11];
            solutions(2,:) = [theta1_1, theta2_12, theta3_12, theta4_12];
            solutions(3,:) = [theta1_2, theta2_21, theta3_21, theta4_21];
            solutions(4,:) = [theta1_2, theta2_22, theta3_22, theta4_22];
            
            % 归一化所有角度
            for i = 1:4
                solutions(i,:) = InverseKinematics.normalizeAngle(solutions(i,:));
            end
        end
        
        % 选择最佳解（与当前关节角度偏差最小）
        function best_theta = chooseBestSolution(arm_model, solutions)
            current_theta = arm_model.getCurrentJointAngle();
            min_error = inf;
            best_idx = 1;
            
            % 计算每组解与当前角度的偏差（平方和最小）
            for i = 1:size(solutions,1)
                error = sum((solutions(i,:) - current_theta).^2);
                if error < min_error
                    min_error = error;
                    best_idx = i;
                end
            end
            
            best_theta = solutions(best_idx,:);
        end
    end
end