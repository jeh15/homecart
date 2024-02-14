function [Th,Nodes,xd_lb,xd_ub] = DevMPC6()

% Generate MPC Constraints
clc; clear; close all
addpath(genpath([pwd '/MPC6/Functions']))

%% Input Parameters
% Time Variables
Th = 1*0.33;          % Time Horizon (lookahead time)

% Problem Nodes
Nodes = 1*10;


% Position Upper/Lower Bounds - [x; y; z], [max, min]
xd_lb = [-2];
xd_ub = [2];  

% xd_lb = [-0.4];
% xd_ub = [0.4];  


% Velocity Upper/Lower Bounds - [x; y; z], [max, min]
vd_lb = [-1.75];
vd_ub = [1.75];

% Cost Function Values
% Design Vector - 
dims_x = size(xd_lb,1);
dims_v = size(vd_lb,1);

% Q = zeros(dims_x*3);
% Q(1:dims_x,1:dims_x) = eye(dims_x)
% last 2000
% Q = .01*[1 0     0 ;
%          0 .0001 0 ;
%          0 0     .0001];

% % Q = 1*[0.5 0 0;
% %        0 1 0;
% %        0 0 0.1];

Q = 0.2*[2 0   0 0;
         0 5*0.2 0 0;
         0 0   0.000001 0;
         0 0   0 0.000001];


R = eye(dims_v).*0.000001;

% Drone Properites
m = 0.032;       % kg
cd = 0.5;        % damping term
Ixx = 2.3851e-5; % kgm^2
Iyy = 2.3851e-5; % kgm^2
Izz = 3.2347e-5; % kgm^2


% Ball properties

golf = 0;
if golf ==1
    % m   = 0.003;       % Mass [kg]
    m   = 0.046;       % Mass [kg]
    rr   = 0.021;       % Radius [m]
    % Jz  = .5*m*rr^2;    % Moment of inertia [kg.m2]
    Jz  = (2/5)*m*rr^2;    % Moment of inertia [kg.m2]
else
    m   = 0.003;       % Mass [kg]
    % m   = 0.046;       % Mass [kg]
    rr   = 0.02;       % Radius [m]
    % Jz  = .5*m*rr^2;    % Moment of inertia [kg.m2]
    Jz  = (2/3)*m*rr^2;    % Moment of inertia [kg.m2]
end
g   = 9.81;        % Gravity [m/s2]
K = m*g / (m+(Jz/rr^2));


% Create the Drone State Space Model 
% Ad = [0 1 0 0;
%       0 0 K 0;
%       0 0 0 1;
%       0 0 0 -31*K];
% 
% 
% 
% Bd = [ 0 ;
%        0 ;
%        0 ;
%        29*K ];


Ad = [0 1 0 0;
      0 0 K 0;
      0 0 0 1;
      0 0 0 -15]; % originally -31 newish -17 BETTER = -15



Bd = [ 0 ;
       0 ;
       0 ;
       10 ]; % originally 29 newish 16 BETTER = 10 


% u - board ang vel step -> equivalent to board-ang-acc = max-step / dt = 0.5/(0.033)
du_max = 0.25; % original 0.5, updated 0.25 
% du_max = 0.3;

%% Setup the Problem Objectives and Constraints
% Develop the Constraint Functions
[x,u,dv] = Constraints(Th,Nodes,Ad,Bd,xd_lb,xd_ub,vd_lb,vd_ub,du_max);

% Develop the Objective Functions
ObjFunc(Q,R,x,u,dv);

end