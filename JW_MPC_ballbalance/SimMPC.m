% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/Functions']))

%% Time Parameters
% Time Variables
Ts = 0.05;         % Computer Update rate (operating time)

%% Setup the MPC Problem

% Initial States - [x; dx; ddx]
qd_i = [0.2; 0; 0];

% Desired State - [x; dx; ddx]
qd_des = [0; 0; 0];

% Create the Drone State Space Model - Also in DevMPC
% Create the Drone State Space Model 
Ad = [0 1 0;
      0 0 1;
      0 0 0];

% Ball properties
m   = 0.003;       % Mass [kg]
R   = 0.017;       % Radius [m]
Jz  = .5*m*R^2;    % Moment of inertia [kg.m2]
g   = 9.81;        % Gravity [m/s2]
K = m*g / (m+(Jz/R^2));

Bd = [0;
      0;
      K];

%% Setup the Problem Objectives and Constraints
Time = 0; i = 1;
[Th,Nodes,xd_lb,xd_ub] = DevMPC6();
qd_prev = qd_i.*ones(size(qd_i,1),Nodes);

%% Simulate the results
% Figure Information
% h1 = figure();
% filename = 'Avoid.gif';
% figure(h1)
% grid minor
% xlim([-2,2]); 
% axis equal
% hold on; 
% Dims = size(Ad,1)/2;
% Obst_num = 1;
% [s.x, s.y, s.z] = sphere(15);

x_array = [];
t_array = [];

% board angle
th = 0;

while Time < 10
    % Create a random control input for the object
    % x_e = qd_i - qo_i;
    % uo = Kp.*x_e(1:size(xd_lb,1)) + Kd.*x_e(size(xd_lb,1)+1:end); 
    
    % Develop the desired Trajectory
    % [qd, ud] = RunMPC6(Th,Ts,Nodes,r_min,r_min2,cost,qd_i,qo_i,qd_des,qd_prev,xd_lb,xd_ub);
    tic
    [qd, ud] = RunMPC6(Th,Nodes,qd_i,qd_des,qd_prev,xd_lb,xd_ub);   
    toc
    
    % new board angle
    th = th + (ud(end)*Ts);

    % Measure the Actual Motion
    [Tsim, qd_sim] = Simulate(Th,Ts,Nodes,qd_i,ud,Ad,Bd,th);
    
    % Animate
    % Animation(i, s, Dims, Obst_num, h1, Ts, r_min, r_min2, qo_i, qd_i, qd, qd_des, filename)
       
    % Update the states, time and iteration
    qd_i = qd_sim(end,1:end).';
    % qo_i = qo_sim(end,1:end).';
    qd_prev = qd;
    Time = Time + Ts; 
    i = i + 1;

    t_array = [t_array,Time];
    x_array = [x_array,qd_i(1)];    

end

plot(t_array,x_array,'.-')

%--------------------------------------------------------------------------
% sim
%--------------------------------------------------------------------------
% function dz = disk_rolling(~,z,parameters,th,dth,ddth)
%     % Rolling disk longitudinal dynamics rolling down an inclined plane.
% 
%     m   = 0.01;            % Mass              [kg]
%     R   = 0.05;
%     Jz  = .5*m*R^2;    % Moment of inertia [kg.m2]
%     g   = 9.81;             % Gravity           [m/s2]
% 
%     % Dynamics
%     dz(1,1) = z(2);
%     dz(2,1) = m*g*sin(th) / (m+Jz/R^2);
% end
