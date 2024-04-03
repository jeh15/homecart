clc;clear;close all;

%%

% ball

x1 = 1.0; % pos
x2 = 0.0; % vel

sig = 0.5;        % covariance matrix with sigma^2 (here: uncertainty considered)




    N      = 10;             % Number of nodes (horizon)
    Th     = 0.1;            % MPC Time horizon (prev 13)

    xmeasure = [x1 x2];      % state
    u0   = 0.6*ones(1,N);    % initial input guess - [u(N), beta]
    beta0   = 0.6;           % initial input guess - [u(N), beta]

%==========================================================================    
%   Cost function
%==========================================================================    

    costQ = 0.0*[1  0 ; 
                 0 .1];
    costR = 1e-8;        % last 1e-2

%==========================================================================    
%   SMPC settings
%==========================================================================    

    sig = 0.5;        % covariance matrix with sigma^2 (here: uncertainty considered)
    x1_limit = 0.9; 
    state = 1;        % 1,2,3,4 - position,velocity,acceleration,jerk

%==========================================================================    
% optimization options
%==========================================================================    
    tol_opt       = 1e-8; 
    options = optimset('Display','off',...
        'TolFun', tol_opt,...
        'MaxIter', 10000000,...
        'Algorithm', 'interior-point',...
        'FinDiffType', 'forward',...
        'RelLineSrchBnd', [],...
        'RelLineSrchBndDuration', 1,...
        'TolConSQP', 1e-6,...
        'MaxFunEvals',1e4);
    
%==========================================================================
%                   System matrices
%==========================================================================    

    Ac = [0 1;
          0 0]; 
    Bc = [0;
          1]; 
    Cc = [1 0];
    Dc = 0;
    [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
    sysA = sysd.A;
    sysB = sysd.B;

%==========================================================================


    params.x1_limit = x1_limit;
    params.sysA = sysA;
    params.sysB = sysB;
    params.Th = Th;
    params.state = state;
    params.costQ = costQ;
    params.costR = costR;

% time = linspace(0.0,0.5,11);
% time = time';

% [x_traj,u_traj] = smpc2_simple(x1,x2);

% x_traj = x_traj;
% % u_traj = out(:,end);
% sgtitle('SMPC')
% subplot(2,2,1)
% plot(time,x_traj(:,1))
% xlabel('time','FontSize',16)
% ylabel('x','FontSize',16)
% title('x','FontSize',20)
% 
% subplot(2,2,2)
% plot(time,x_traj(:,2))
% xlabel('time','FontSize',16)
% ylabel('dx','FontSize',16)
% title('dx','FontSize',20)
% 
% subplot(2,2,3)
% plot(time(1:end-1),u_traj(1:end-1))
% xlabel('time','FontSize',16)
% ylabel('u','FontSize',16)
% title('u','FontSize',20)

% subplot(2,2,4)
% plot(time,x_traj(:,4))
% xlabel('time','FontSize',16)
% ylabel('board velocity (rad/s)','FontSize',16)
% title('board velocity','FontSize',20)


%%


% Start of the NMPC iteration

mpciter = 0;
xx=[];
tt=[];
mpciterations = 100;
Th     = 0.1;            % MPC Time horizon (prev 13)

t0 = 0.0;

bb = zeros(1,mpciterations);

while(mpciter < mpciterations)

    [x_traj,u_traj] = smpc2_simple(x1,x2);


    % [u_new, V_current, exitflag, output] = solveOptimalControlProblem ...
    %     (runningcosts, terminalcosts, constraints, ...
    %     terminalconstraints, linearconstraints, system, cov_propagation, ...
    %     N, t0, x0, u0, T, ...
    %     sig, beta, params, ...
    %     atol_ode_sim, rtol_ode_sim, tol_opt, options, type);



    u0 = u_traj(1:end-1);

    bb(mpciter+1) = u_traj(end);
    
    x0 = [x1,x2];

    [tmeasure, xmeasure] = applyControl(@system, Th, x0, u0, ...
        sig, params);

    x1 = xmeasure(1);
    x2 = xmeasure(2);

    mpciter = mpciter+1

    xx = [xx,xmeasure(1)];
    % tt = [tt,tmeasure];
end

plot(xx,'.-',"LineWidth",3)
hold on
plot(-0.1*ones(length(xx),1),'r',"LineWidth",3)
hold on
plot(0.0*ones(length(xx),1),'k--',"LineWidth",3)
grid on
title('smpc');
xlabel('Time (s)');
ylabel('Position (m)');
%     axis([-.5 totalTime+1 -.4 1.1])
text( 0.5 , -0.1-0.02, 'Chance constraint','FontSize',20)
text( 0.5 , -0.0-0.02, 'Goal','FontSize',20)
ax = gca; 
ax.FontSize = 16;

figure(2)
plot(bb)

%%

function y = system(x, u, Th, apply_flag, sig, params)
% apply_flag: 1) noise is applied for real system; 0) no noise for prediction


Ac = [0 1;
      0 0]; 
Bc = [0;
      1]; 
Cc = [1 0];
Dc = 0;
[sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),0.1,'zoh');
sysA = sysd.A;
sysB = sysd.B;

%==========================================================================


% params.x1_limit = x1_limit;
params.sysA = sysA;
params.sysB = sysB;


A = params.sysA;
B = params.sysB;
K = [0,0];

y = A*x'+B*(u(1,1) - K*[x(1); x(2)]);


if apply_flag == 1
%         D = [1 0; 0 1];
    D = [0 0; 0 1];
    w = [0;0];
    % Gaussian noise with variance sig^2
    w(1) = normrnd(0,sig);
    w(2) = normrnd(0,sig);
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
