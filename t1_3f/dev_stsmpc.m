function [A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol] = dev_stsmpc()

% clc;clear;close all;

addpath('/home/orl/Downloads/casadi-3.6.6-linux64-matlab2018b/')



% system matrices
%================
A = [0   1      0     0; 
     0   0      5.886 0;
     0   0      0     1;
     0   0      0   -15];

%===============
B1= [0;
     0;
     0;
    10];
    
%===============

title_text = 'True mode: Non-Min-phase, Modes: Min/NonMin';
% B real

B_real = B1;

%===============
% b modes

B = [0.8*B1,10*B1];
%================

% 2 modes B(2,1) = 1,2 (modes)

% sampling time
%================
Ts = 0.2;
%================

% cost function matrices
% Q = diag([1;0.1]);
%================
Q = diag(1000*[1;1;1;1]);
R = 1;
%================

% dimensions
n_x = size(A,1); % state
% n_u = size(B,2); % input
n_u = 1; % input
n_m = 2; % number of modes

% true values of gamma per mode
%================
% true_gamma = [1;0;0;0;0.9];
true_gamma = [1;0];
b1 = 2;
b2 = 1;

bx = [0;
      0;
      0;
      10];

%================

% mean and variance of noise w(k)
%================
mW = zeros(n_x,1);
% varW = diag([0.00001;0.00001;0.00001;0.00001]);
varW = diag([0.00001;0.00001;0.00001;0.00001]);
ivarW = inv(varW);
%================

% prior distributions (gamma^M and M)
%================
mu_gamma_M(:,1) = [1;1];
var_gamma_M(:,1) = [0.01;0.01];
p_M(:,1) = [0.5; 0.5];
%================


% initial state
% x(:,1) = 1*ones(n_x,1);
x(:,1) = [0;0;0;0];

% time horizon
%================
% T = 80*10*2; % 1600
T = 1000; % 1600
%================

% control horizon
%================
N = 20; % 20
%================

% fault step
%================
% k_fault = 100*3; % 21
k_fault = T/2; % 5
% k_fault = 10; % 21
%================

% reference trajectory
%================
delay = 0;
% ref = [zeros(k_fault+delay,1); ones(T+N-(k_fault+delay),1)*0]; %50
ref = [zeros(k_fault+delay,1); ones(T+N-(k_fault+delay),1)*0]; %50

% number of samples
N_s = 2;

% lookahead steps
L = 1;

% noise samples
w_ = chol(varW)*randn(n_x,T);

simulation = struct;

my_mvnpdf = @(x,mu,var) (1/( sqrt((2*pi)^(length(x))*det(var)) ) )*exp( -0.5*(x - mu)'*inv(var)*(x - mu) );


% construct controller with structural + parametric uncertainty

sol = dual_structural_controller(N,N_s,n_m,n_x,n_u,L,A,B,Ts,varW,ivarW,Q,R,my_mvnpdf);





end
