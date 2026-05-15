%% 机械臂建模
% 定义各个连杆以及关节类型，默认为转动关节
%          theta      d       a      alpha
L1=Link([    0         0       0  pi/2], 'standard');
L2=Link([    0         0       0.39  0], 'standard');
L3=Link([     0           0  0.36      0], 'standard');
L4=Link([     0        0   0.14     0], 'standard');

b=isrevolute(L1);
robot=SerialLink([L1,L2,L3,L4],'name','Irvingao Arm'); % 将四个连杆组成机械臂
robot.name='4DOF Robotic Arm';
robot.display();
view(3);
robot.teach();
robot.plot([0 pi/2 0 0]);
