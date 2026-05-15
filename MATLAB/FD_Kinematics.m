clc;
clear;
%% 机械臂建模
% 定义各个连杆以及关节类型，默认为转动关节
%          theta      d       a      alpha
L1=Link([    0         0       0  pi/2], 'standard');
L2=Link([    0         0       0.39  0], 'standard');
L3=Link([     0           0  0.36      0], 'standard');
L4=Link([     0        0   0.22     0], 'standard');
b=isrevolute(L1);
robot=SerialLink([L1,L2,L3,L4],'name','Irvingao Arm'); % 将四个连杆组成机械臂
robot.name='4DOF Robotic Arm';
robot.display();

%% 轨迹规划
% 初始值及目标值
init_ang=[0 0 0 0];
targ_ang=[0, -pi/6, -pi/5, pi/6];
step=200;

[q,qd,qdd]=jtraj(init_ang,targ_ang,step); %关节空间规划轨迹，得到机器人末端运动的[位置，速度，加速度]
T0=robot.fkine(init_ang); % 正运动学解算
Tf=robot.fkine(targ_ang);
subplot(2,4,3); i=1:4; plot(q(:,i)); title("位置"); grid on;
subplot(2,4,4); i=1:4; plot(qd(:,i)); title("速度"); grid on;
subplot(2,4,7); i=1:4; plot(qdd(:,i)); title("加速度"); grid on;

Tc=ctraj(T0,Tf,step); % 笛卡尔空间规划轨迹，得到机器人末端运动的变换矩阵
Tjtraj=transl(Tc);
subplot(2,4,8); plot2(Tjtraj, 'r');
title('p1到p2直线轨迹'); grid on;
subplot(2,4,[1,2,5,6]);
plot3(Tjtraj(:,1),Tjtraj(:,2),Tjtraj(:,3),"b"); grid on;
hold on;
view(3); % 解决robot.teach()和plot的索引超出报错
qq=robot.ikine(Tc, 'q0',[0 0 0 0], 'mask',[1 1 1 1 0 0]); % 逆解算
robot.plot(qq);
