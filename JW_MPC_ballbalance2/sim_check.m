clc;clear;close all

addpath("Functions/Dynamics/")

Th = 0.5;
Ts = 0.05;
Nodes = 15;
qd_i = [0,0,0];
ud = 10*ones(1,Nodes);
Ad = [0 1 0;
      0 0 1;
      0 0 0];
% Ball properties
m   = 0.003;       % Mass [kg]
rr  = 0.017;       % Radius [m]
Jz  = .5*m*rr^2;    % Moment of inertia [kg.m2]
g   = 9.81;        % Gravity [m/s2]
K = m*g / (m+(Jz/rr^2));
Bd = [ 0 ;
       0 ;
       K ];

[Tsim,qd_sim] = Simulate(Th,Ts,Nodes,qd_i,ud,Ad,Bd)