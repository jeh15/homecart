clc;clear;close all;

%%

% ball

x1 = 1.0; % pos
x2 = 0.0; % vel


time = linspace(0.0,0.5,11);

[x_traj,u_traj] = smpc2_simple(x1,x2);

x_traj = x_traj;
% u_traj = out(:,end);
sgtitle('SMPC')
subplot(2,2,1)
plot(time,x_traj(:,1))
xlabel('time','FontSize',16)
ylabel('ball position (m)','FontSize',16)
title('ball position','FontSize',20)

subplot(2,2,2)
plot(time,x_traj(:,2))
xlabel('time','FontSize',16)
ylabel('ball velocity (m/s)','FontSize',16)
title('ball velocity','FontSize',20)

subplot(2,2,3)
plot(time,x_traj(:,3))
xlabel('time','FontSize',16)
ylabel('board position (rad)','FontSize',16)
title('board position','FontSize',20)

subplot(2,2,4)
plot(time,x_traj(:,4))
xlabel('time','FontSize',16)
ylabel('board velocity (rad/s)','FontSize',16)
title('board velocity','FontSize',20)

