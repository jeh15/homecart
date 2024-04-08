clc;clear;close all;

M = readmatrix("data/mpc_1dlearn_2024_02_22-06_09_37_PM.csv");

%%
time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
board_pos = M(4,:);
board_vel = M(5,:);
board_targ_vel = M(6,:);
kf2_pos = M(7,:);
kf2_vel = M(8,:);
kf2_acc = M(9,:);
kmodel = M(10,:);
%%
plot(time,kmodel,time,0.01*ones(size(time)),time,10.0*ones(size(time)))