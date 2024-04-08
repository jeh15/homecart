clc;clear;close all;

%%

%==========================================================================    
%   optim settings
%==========================================================================    

x1 = 0.05; % pos
x2 = 0.0; % vel
x3 = 0.0; % pos
x4 = 0.0; % vel

N      = 10;             % Number of nodes (horizon)
Th     = 0.1;            % MPC Time horizon (prev 13)

xmeasure = [x1 x2,x3,x4];      % state
u0   = 0.6*ones(1,N);    % initial input guess - [u(N), beta]
beta0   = 0.6;           % initial input guess - [u(N), beta]

%==========================================================================    
%   Cost function
%==========================================================================    

    costQ = 0.0*[1  0  0  0; 
                 0 .1  0  0;
                 0  0 .1  0;
                 0  0  0 .1];
    costR = 1e-8;        % last 1e-2

%==========================================================================    
%   SMPC settings
%==========================================================================    

sig = 0.01;        % covariance matrix with sigma^2 (here: uncertainty considered)
x1_limit = [0.9,1.1]; 
state = 1;        % 1,2,3,4 - position,velocity,acceleration,jerk

%==========================================================================
%                   System matrices
%==========================================================================    

golf = 0;
    if golf ==1
        m   = 0.046;       % Mass [kg]
        rr   = 0.021;       % Radius [m]
        Jz  = (2/5)*m*rr^2;    % Moment of inertia [kg.m2]
    else
        m   = 0.003;       % Mass [kg]
        rr   = 0.02;       % Radius [m]
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
    [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
    sysA = sysd.A;
    sysB = sysd.B;

%==========================================================================


% params.x1_limit = x1_limit;
params.sysA = sysA;
params.sysB = sysB;
params.Th = Th;
params.state = state;
params.costQ = costQ;
params.costR = costR;


%%


% Start of the NMPC iteration

mpciter = 0;
xx=[];
tt=[];
mpciterations = 25;
Th     = 0.1;            % MPC Time horizon (prev 13)

t0 = 0.0;

bb = zeros(1,mpciterations);

t_elapsed = zeros(1,mpciterations);

fails = [];

while(mpciter < mpciterations)
    
    tic
    [x_traj,u_traj,exitflag] = smpc2_4d(x1,x2,x3,x4,0.0);
    t_elapsed(mpciter+1) = toc;

    u0 = u_traj(1:end-1);

    bb(mpciter+1) = u_traj(end);
    
    x0 = [x1,x2,x3,x4];

    [tmeasure, xmeasure] = applyControl(@system, Th, x0, u0, ...
        sig, params);

    x1 = xmeasure(1);
    x2 = xmeasure(2);
    x3 = xmeasure(3);
    x4 = xmeasure(4);

    mpciter = mpciter+1;
    if (exitflag <=0)
        disp(strcat(num2str(mpciter),'-- optim failed { 0 : itermax // -2 : no feasible point found } -> ',num2str(exitflag)))
        fails = [fails, mpciter];

    end

    xx = [xx,xmeasure(1)];
    % tt = [tt,tmeasure];
end

t_array = Th:Th:((mpciterations)*Th);
%%
subplot(3,1,1)
plot(t_array,xx,'.-',"LineWidth",3)
hold on
plot(t_array,0.9*ones(length(xx),1),'r',"LineWidth",2)
% hold on
% plot(t_array,0.0*ones(length(xx),1),'k--',"LineWidth",3)
grid on
title('smpc');
xlabel('Time (s)');
ylabel('Position (m)');
%     axis([-.5 totalTime+1 -.4 1.1])
text( 1.5 , 0.9+0.2, 'Chance constraint','FontSize',20)
% text( 0.5 , -0.0-0.02, 'Goal','FontSize',20)
ax = gca; 
ax.FontSize = 16;

subplot(3,1,2)
plot(t_array,bb,'.-','LineWidth',2,'MarkerSize',12)
grid on
title('beta');
xlabel('Time (s)');
ylabel('beta');
ax = gca; 
ax.FontSize = 16;

subplot(3,1,3)
plot(t_array,t_elapsed,'.-','LineWidth',2,'MarkerSize',12)
hold on
plot(Th*fails,zeros(size(fails)),'or','MarkerSize',12)
grid on
title('t elapsed');
xlabel('Time (s)');
ylabel('t elapsed (s)');
ax = gca; 
ax.FontSize = 16;


%%

function y = system(x, u, Th, apply_flag, sig, params)
% apply_flag: 1) noise is applied for real system; 0) no noise for prediction


% golf = 0;
% if golf ==1
%     m   = 0.046;       % Mass [kg]
%     rr   = 0.021;       % Radius [m]
%     Jz  = (2/5)*m*rr^2;    % Moment of inertia [kg.m2]
% else
%     m   = 0.003;       % Mass [kg]
%     rr   = 0.02;       % Radius [m]
%     Jz  = (2/3)*m*rr^2;    % Moment of inertia [kg.m2]
% end
% g   = 9.81;        % Gravity [m/s2]
% K = m*g / (m+(Jz/rr^2));
% 
% Ac = [0 1 0 0;
%       0 0 K 0;
%       0 0 0 1;
%       0 0 0 -15]; % originally -31 newish -17 BETTER = -15
% Bc = [ 0 ;
%        0 ;
%        0 ;
%        10 ]; % originally 29 newish 16 BETTER = 10
% 
% Cc = [1 0 0 0];
% Dc = 0;
% [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
% sysA = sysd.A;
% sysB = sysd.B;

%==========================================================================


% params.x1_limit = x1_limit;
% params.sysA = sysA;
% params.sysB = sysB;


A = params.sysA;
B = params.sysB;
K = [0,0,0,0];

y = A*x'+B*(u(1,1) - K*[x(1); x(2); x(3); x(4)]);


if apply_flag == 1
%         D = [1 0; 0 1];
    D = [0 0 0 0;
         0 1 0 0;
         0 0 0 0;
         0 0 0 0];
    w = [0;0;0;0];
    % Gaussian noise with variance sig^2
    w(1) = normrnd(0,sig);
    w(2) = normrnd(0,sig);
    w(3) = normrnd(0,sig);
    w(4) = normrnd(0,sig);    
    % determine next state with uncertainty
    y = y + D*w;
end

y = y';    
end


%%

function [tapplied, xapplied] = applyControl(system, Th, x0, u, ...
                                sig, params)
    apply_flag = 1; % for prediction, no noise is applied (=0); for actual system noise is applied (=1)
    xapplied = dynamic(system, Th, x0, u(:,1), ...
                       apply_flag, sig, params);
    tapplied = 0;
end

%%

function [x] = dynamic(system, Th, ...
             x0, u, apply_flag, sig, params)
    x = system(x0, u, Th, apply_flag, sig, params);
end
