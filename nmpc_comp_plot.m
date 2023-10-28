clc
clear
close all

M = readmatrix("data/data2_2023_10_27-07_55_06_PM.csv");

time = M(1,:);
ball_pos = M(2,:);
arm_acc = M(3,:);
ball_vel = M(4,:);
ball_acc = M(5,:);
ball_jerk = M(6,:);

u_traj = readmatrix("data/u2.csv");

%% plot states

subplot(4,1,1)
plot(ball_pos(1:25),'.-')

subplot(4,1,2)
plot(ball_vel(1:25),'.-')

subplot(4,1,3)
plot(ball_acc(1:25),'.-')

subplot(4,1,4)
plot(ball_jerk(1:25),'.-')

figure(2)
plot(arm_acc(1:25),'.-')


%%

k = 18;

a = [ball_pos(k),ball_vel(k),ball_acc(k),ball_jerk(k)];

tic
u = nmpc_test1(a(1),a(2),a(3),a(4));
toc

tic
u2 = nmpc2(a(1),a(2),a(3),a(4));
toc
figure(3)
plot(u,'.-b')
hold on
plot(u2+.0,'.-m')