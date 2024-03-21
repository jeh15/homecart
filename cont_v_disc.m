clc;clear;close all;
%%

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


Ac = [0 1 0 0;
      0 0 K 0;
      0 0 0 1;
      0 0 0 -15]; % originally -31 newish -17 BETTER = -15
Bc = [ 0 ;
       0 ;
       0 ;
       10 ]; % originally 29 newish 16 BETTER = 10

Cc = [1 0 0 0];
Dc = 0;


%% cont

x_init = [0,0,0,0];
x_now = x_init';

xc = [];

u = 0.1;

t0 = 0.0;
tf = 1.0;
nt = 20;
dt = (tf-t0)/nt;
t = linspace(t0,tf,nt);

for i=1:nt
    dx = (Ac*x_now) + (Bc*u);
    x_now = x_now + dx*dt;
    xc = [xc,x_now];
end

%% disc

x_init = [0,0,0,0];
x_now = x_init';

xd = [];


Th = dt;

[sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
Ad = sysd.A;    
Bd = sysd.B;    

% u = 0.1;
% 
% t0 = 0.0;
% tf = 1.0;
% nt = 10;
% dt = (t0-tf)/nt;
% t = linspace(t0,tf,nt);

for i=1:nt
    x_now = (Ad*x_now) + (Bd*u);
    xd = [xd,x_now];
end


%%
% subplot(3,1,1)
plot(t,xc(4,:),'.-r',t,xd(4,:),'.-b')
legend('cont','disc')

