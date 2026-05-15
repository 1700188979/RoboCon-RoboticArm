classdef ForwardKinematics
    methods (Static)
        % 计算单个关节的DH变换矩阵
        function T = getDHTransform(alpha, a, d, theta)
            ca = cos(alpha);
            sa = sin(alpha);
            ct = cos(theta);
            st = sin(theta);
            
            T = [ct,    -st*ca,   st*sa,   a*ct;
                 st,     ct*ca,  -ct*sa,   a*st;
                 0,       sa,      ca,      d;
                 0,        0,       0,      1];
        end
        
        % 正运动学求解：输入机械臂模型，输出末端位姿矩阵(4x4)
        function T_end = calculateEndPose(arm_model)
            dh = arm_model.getDHParam();
            joint_num = arm_model.joint_num;
            T_total = eye(4);  % 初始变换矩阵（齐次坐标）
            
            % 累积所有关节的变换矩阵
            for i = 1:joint_num
                T_i = ForwardKinematics.getDHTransform( ...
                    dh.alpha(i), dh.a(i), dh.d(i), dh.theta(i) ...
                );
                T_total = T_total * T_i;
            end
            
            T_end = T_total;
        end
        
        % 提取末端位置（x,y,z）
        function [x, y, z] = getEndPosition(arm_model)
            T_end = ForwardKinematics.calculateEndPose(arm_model);
            x = T_end(1,4);
            y = T_end(2,4);
            z = T_end(3,4);
        end
    end
end