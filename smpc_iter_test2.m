clc;clear;close all;

%%

% ball

x1 = 0.01; % pos
x2 = 0.0; % vel

% board
x3 = 0.2; % pos
x4 = 0.0; % vel

% target
xt = 0.0; % pos

time = linspace(0.0,0.5,11);

% tic
% out = nmpc_new(x1,x2,x3,x4,xt);
% plot(time,out)
% toc



[x_traj,u_traj] = smpc2_4d(x1,x2,x3,x4,xt)

x_traj = x_traj;
% u_traj = out(:,end);
subplot(2,2,1)
sgtitle('NMPC')
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


[x_traj,u_traj] = smpc(x1,x2,x3,x4,xt);

x_traj = x_traj;
% u_traj = out(:,end);
figure(2)
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

