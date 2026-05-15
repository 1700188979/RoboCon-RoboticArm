classdef RoboticArmModel
    % 修复：用兼容所有版本的语法声明公共属性
    properties
        % DH参数表（4自由度机械臂）
        DH = struct(...
            'a', [0.0, 0.390, 0.360, 0.140], ...  % 连杆长度(m)
            'd', [0.0, 0.0, 0.0, 0.0], ...        % 连杆偏距(m)
            'alpha', [pi/2, 0, 0, 0], ...         % 连杆扭角(rad)
            'theta', [0.0, 0.0, 0.0, 0.0] ...     % 当前关节角度(rad)（4个关节，长度=4）
        );
        joint_num = 4;  % 关节数量
    end
    
    methods
        % 构造函数：初始化关节角度
        function obj = RoboticArmModel(init_theta)
            if nargin > 0 && length(init_theta) == obj.joint_num
                obj.DH.theta = init_theta;  % 统一DH大小写，避免引用错误
            end
        end
        
        % 更新关节角度
        function updateJointAngle(obj, new_theta)
            if length(new_theta) == obj.joint_num
                obj.DH.theta = new_theta;
            else
                warning('关节角度数量不匹配！需输入4个关节角度');
            end
        end
        
        % 获取当前关节角度
        function theta = getCurrentJointAngle(obj)
            theta = obj.DH.theta;
        end
        
        % 获取DH参数
        function dh_param = getDHParam(obj)
            dh_param = obj.DH;
        end
    end
end