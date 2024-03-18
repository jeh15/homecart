clc;clear;close all;

%%

% ball

x1 = 0.1; % pos
x2 = 0.0; % vel

% board
x3 = 0.2; % pos
x4 = 0.0; % vel

% target
xt = 0.0; % pos

time = linspace(0.0,0.05,11);

tic
out = nmpc_new(x1,x2,x3,x4,xt);
plot(time,out)
toc



