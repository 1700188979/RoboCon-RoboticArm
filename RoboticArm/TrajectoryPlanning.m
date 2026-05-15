
% 附加：轨迹规划工具类（用于动画平滑运动）
classdef TrajectoryPlanning
    methods (Static)
        % 五次多项式轨迹规划
        function theta_t = quinticPolynomial(t, T, theta0, d_theta0, dd_theta0, thetaf, d_thetaf, dd_thetaf)
            if T <= 0
                theta_t = theta0 * ones(size(t));
                return;
            end
            
            % 多项式系数计算
            T2 = T^2; T3 = T^3; T4 = T^4; T5 = T^5;
            a0 = theta0;
            a1 = d_theta0;
            a2 = 0.5 * dd_theta0;
            a3 = (20*(thetaf-theta0) - (8*d_thetaf + 12*d_theta0)*T - (3*dd_theta0 - dd_thetaf)*T2) / (2*T3);
            a4 = (30*(theta0-thetaf) + (14*d_thetaf + 16*d_theta0)*T + (3*dd_theta0 - 2*dd_thetaf)*T2) / (2*T4);
            a5 = (12*(thetaf-theta0) - (6*d_thetaf + 6*d_theta0)*T - (dd_theta0 - dd_thetaf)*T2) / (2*T5);
            
            % 计算每个时间点的角度
            theta_t = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
        end
    end
end