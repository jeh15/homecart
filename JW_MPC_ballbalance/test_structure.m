% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/Functions']))

%% Setup the MPC Problem

% Initial States - [x; dx; ddx]
qd_i = [-.2; 0; 0];

% Desired State - [x; dx; ddx]
qd_des = [-.0; 0; 0];


[Th,Nodes,xd_lb,xd_ub] = DevMPC6();


[qd, ud] = RunMPC6(Th,Nodes,qd_i,qd_des,xd_lb,xd_ub);   

t_traj = linspace(0,Th,Nodes);

subplot(4,1,1)
plot(t_traj, qd(1,:))

subplot(4,1,2)
plot(t_traj, qd(2,:))

subplot(4,1,3)
plot(t_traj, qd(3,:))

subplot(4,1,4)
plot(t_traj, qd(3,:))

figure(2)
plot(t_traj,ud)