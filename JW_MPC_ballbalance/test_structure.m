% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/Functions']))

%% Setup the MPC Problem

% Initial States - [x; dx; ddx]
qd_i = [-.5; 0; 0];

% Desired State - [x; dx; ddx]
qd_des = [-.6; 0; 0];


[Th,Nodes,xd_lb,xd_ub] = DevMPC6();


[qd, ud] = RunMPC6(Th,Nodes,qd_i,qd_des,xd_lb,xd_ub);   

linspace(0,Th,Nodes)

ud

