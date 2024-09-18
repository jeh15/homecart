clc
clear
close all

addpath('/home/vivek/casadi-3.6.5-linux64-matlab2018b/')

% double integrator model

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

B2= [0;
     1;
     0;
    10];

    
%===============

title_text = 'True mode: Non-Min-phase, Modes: Min/NonMin';
% B real

B_real = B1;

%===============
% b modes

B = [B1,B2];
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
%================

% number of samples
N_s = 2;

% lookahead steps
%================
L = 1; % initially 1
%================

% noise samples
w_ = chol(varW)*randn(n_x,T);

simulation = struct;

% gaussian density function
my_mvnpdf = @(x,mu,var) (1/( sqrt((2*pi)^(length(x))*det(var)) ) )*exp( -0.5*(x - mu)'*inv(var)*(x - mu) );

% construct controller with structural + parametric uncertainty
sol = dual_structural_controller(N,N_s,n_m,n_x,n_u,L,A,B,Ts,varW,ivarW,Q,R,my_mvnpdf);

progressbar

for t=1:T
    tic

    progressbar(t/T)

    for i=1:L
        gamma_M{i} = randn(1,(N_s*n_m)^i);
        w{i} = mW + chol(varW)*randn(n_x,(N_s*n_m)^i);
    end
    wunwrap = [];
    gammaunwrap = [];
    for i=1:L
        wunwrap = [wunwrap, w{i}(:)'];
        gammaunwrap = [gammaunwrap,gamma_M{i}(:)'];
    end
    
    param = [ref(t:t+N-1)', x(:,t)', p_M(:,t)', mu_gamma_M(:,t)',var_gamma_M(:,t)',wunwrap,gammaunwrap];
    tic
    r = sol('x0',0,'lbx',-0.2,'ubx',+0.2,'p',param);
    toc

    x_opt = r.x;

    u(:,t) = full(x_opt(1:n_u));

    % closedloop_cost(t) = (x(:,t) - [0;ref(t)])'*Q*(x(:,t) - [0;ref(t)]) + u(:,t)'*R*u(:,t);
    
    % update closed-loop dynamics based on the mode in which the system is operating
    if t < k_fault
        x(:,t+1) = (A*Ts + eye(n_x))*x(:,t) + Ts*B_real*true_gamma(1)*u(:,t) + Ts*w_(:,t);

        normlz = 0;
        for model=1:n_m
            x0 = x(:,t+1) - (A*Ts + eye(n_x))*x(:,t);

            var_gamma_M(model,t+1) = 1./( 1./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*(Ts*B(:,model)*u(:,t)) );
            % mu_gamma_M(model,t+1) = var_gamma_M(model,t+1)*( mu_gamma_M(model,t)./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*x0 );
            mu_gamma_M(model,t+1) = 1;

            mu = Ts*B(:,model)*u(:,t)*mu_gamma_M(model,t);
            var = (Ts*B(:,model)*u(:,t))*(var_gamma_M(model,t))*(Ts*B(:,model)*u(:,t))' + varW;

            p_M(model,t+1) = my_mvnpdf(x0,mu,var)*p_M(model,t);
            normlz = normlz + p_M(model,t+1);
        end
        p_M(:,t+1) = p_M(:,t+1)/normlz;

    else
        % x(:,t+1) = (A*Ts + eye(n_x))*x(:,t) + Ts*B(:,5)*true_gamma(2)*u(:,t) + Ts*w_(:,t);
        x(:,t+1) = (A*Ts + eye(n_x))*x(:,t) + Ts*B_real*u(:,t) + Ts*w_(:,t);

        normlz = 0;
        for model=1:n_m
            x0 = x(:,t+1) - (A*Ts + eye(n_x))*x(:,t);

            var_gamma_M(model,t+1) = 1./( 1./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*(Ts*B(:,model)*u(:,t)) );
            % mu_gamma_M(model,t+1) = var_gamma_M(model,t+1)*( mu_gamma_M(model,t)./var_gamma_M(model,t) + (Ts*B(:,model)*u(:,t))'*ivarW*x0 );
            mu_gamma_M(model,t+1) = 1;

            mu = Ts*B(:,model)*u(:,t)*mu_gamma_M(model,t);
            var = (Ts*B(:,model)*u(:,t))*(var_gamma_M(model,t))*(Ts*B(:,model)*u(:,t))' + varW;

            p_M(model,t+1) = mvnpdf(x0,mu,var)*p_M(model,t);
            normlz = normlz + p_M(model,t+1);
        end
        p_M(:,t+1) = p_M(:,t+1)/normlz;

    end
    
    % avoiding mode saturation by capping max and min probabilities to 0.95
    % and 0.05, respectively
    % p_M(p_M > 0.95) = 0.95;
    % p_M(p_M < 0.05) = 0.05;

    p_M(p_M > 0.99) = 0.99;
    p_M(p_M < 0.01) = 0.01;

toc
end

simulation.x = x;
simulation.u = u;
simulation.p_M = p_M;
% simulation.closedloop_cost = closedloop_cost;
simulation.w_ = w_;
simulation.mu_gamma_M = mu_gamma_M;
simulation.var_gamma_M = var_gamma_M;



% plot([0:Ts:T*Ts],simulation.x')
% ax = gca;
% ax.FontSize = 30;
% title('Dual Control SMPC on a 1D double integrator model with fault at t=60s','Interpreter','latex')
% xlabel('time(s)','Interpreter','latex')
% ylabel('position(m) \& velocity(ms-1)','Interpreter','latex')
% ax.TickLabelInterpreter = 'latex';
% text((k_fault-50)*0.2,0.3,'Mode switch','Interpreter','latex','FontSize',20)
% hold on
% plot(k_fault*0.2,0,'or','MarkerSize',20)
% legend('pos','vel','mode switch','Interpreter','latex')


% second plot
% ax = gca;
% ax.FontSize = 30;
% title('Dual Control SMPC on a 1D double integrator model [5 modes B(2):1->4.5] at t=60s','Interpreter','latex')
% xlabel('time(s)','Interpreter','latex')
% ylabel('mode probability','Interpreter','latex')
% ax.TickLabelInterpreter = 'latex';
% text((k_fault-50)*0.2,0.3,'Mode switch','Interpreter','latex','FontSize',20)
% hold on
% plot(k_fault*0.2,0,'or','MarkerSize',20)
% legend('mode 1','mode 2','mode 3','mode 4','mode 5','mode switch','Interpreter','latex')
% 
% hold on
% plot([60,60],[0,1],'--')
% ylim([0,1])


%% test subplot 

subplot(3,1,1)
plot([0:Ts:T*Ts],simulation.x')
title(title_text,'Interpreter','latex')
xlabel('time(s)','Interpreter','latex')
ylabel('state trajectories','Interpreter','latex')
legend('$x$','$\dot{x}$','$\theta$','$\dot{\theta}$','Interpreter','latex')
ax = gca;
ax.FontSize = 15;
ax.TickLabelInterpreter = 'latex';

subplot(3,1,2)
plot([0:Ts:T*Ts],simulation.p_M')
xlabel('time(s)','Interpreter','latex')
ylabel('mode probability','Interpreter','latex')
legend('mode 1','mode 2','mode 3','mode 4','mode 5','Interpreter','latex')
ax = gca;
ax.FontSize = 15;
ax.TickLabelInterpreter = 'latex';

subplot(3,1,3)
plot([0:Ts:T*Ts],simulation.mu_gamma_M')
xlabel('time(s)','Interpreter','latex')
ylabel('gamma mean','Interpreter','latex')
legend('mode 1','mode 2','mode 3','mode 4','mode 5','Interpreter','latex')

ax = gca;
ax.FontSize = 15;
ax.TickLabelInterpreter = 'latex';



