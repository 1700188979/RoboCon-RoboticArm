% 基于Robotics Toolbox建立DH模型
L(1) = Link('d',0.4318,'a',0,'alpha',pi/2);  % 基座
L(2) = Link('d',0,'a',0.39225,'alpha',0);
L(3) = Link('d',0.1333,'a',0.13585,'alpha',0);
L(4) = Link('d',0.4331,'a',0,'alpha',pi/2);
L(5) = Link('d',0.08535,'a',0,'alpha',-pi/2);
L(6) = Link('d',0.3155,'a',0,'alpha',0);

robot = SerialLink(L,'name','PUMA560');
robot.plot([0 0 0 0 0 0]);  % 初始姿态显示
% 创建GUI组件
f = uifigure('Name','机械臂控制台','Position',[100 100 800 600]);
ax = uiaxes(f,'Position',[0.1 0.3 0.7 0.6]);  % 3D绘图区

% 输入控件
uicontrol('Style','text','String','目标X:','Position',[20 500 50 25]);
x_input = uieditfield(f,'numeric','Position',[80 500 100 25]);

% 控制按钮
btn = uibutton(f,'Text','执行运动','Position',[300 450 100 30],...
    'ButtonPushedFcn',@(btn,event) moveArm());

% 状态显示
status = uilabel(f,'Text','就绪','Position',[300 400 200 25]);
function moveArm()
    % 获取目标坐标
    x = str2double(x_input.Value);
    targetPose = trvec2tform([x, 0, 0.5]);  % Z轴抬升0.5m
    
    % 逆运动学求解
    ik = inverseKinematics('RigidBodyTree',robot);
    weights = [1 1 1 1 1 1];
    [configSoln, solnInfo] = ik(robot.homeConfiguration, targetPose, weights);
    
    % 运动轨迹规划
    traj = trapveltraj(configSoln.JointPosition, 100);
    
    % 实时动画更新
    for i = 1:size(traj,1)
        robot.plot(traj(i,:));
        drawnow;
    end
end
