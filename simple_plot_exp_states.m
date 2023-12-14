clc;clear;close all;

M = readmatrix("data/data_movel_2d_2023_12_13-09_36_32_PM.csv");


time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
ball_acc = M(4,:);
target_vel = M(5,:);

pt = -0.4626289703502494;
target = pt*ones(size(time));

subplot(4,1,1)
plot(time,ball_pos,time,target)
title("Ball Position")
xlabel("Time (s)")
ylabel("Position (m)")

subplot(4,1,2)
plot(time,ball_vel)
title("Ball Velocity")
xlabel("Time (s)")
ylabel("Velocity (m/s)")

subplot(4,1,3)
plot(time,ball_acc)
title("Ball Acceleration")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")

subplot(4,1,4)
plot(time,target_vel,time,zeros(size(time)))
title("Target Velocity")
xlabel("Time (s)")
ylabel(" target Velocity (m/s)")

