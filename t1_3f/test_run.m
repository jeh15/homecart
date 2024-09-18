clc;clear;close all;

%%

[A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol] = dev_stsmpc();



u = 0;


for t=1:T
    tic

    [u,p_M, mu_gamma_M,var_gamma_M] = run_stsmpc1(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t);

    
    % update closed-loop dynamics based on the mode in which the system is operating
    if t < k_fault
%--------------- sim --------------------------------------------------------        
        x(:,t+1) = (A*Ts + eye(n_x))*x(:,t) + Ts*B(:,1)*u(:,t) + Ts*w_(:,t);
%--------------- sim --------------------------------------------------------        
    else
%--------------- sim --------------------------------------------------------                
        x(:,t+1) = (A*Ts + eye(n_x))*x(:,t) + Ts*B(:,1)*u(:,t) + Ts*w_(:,t);
%--------------- sim --------------------------------------------------------        


    end
    
    [u,p_M, mu_gamma_M,var_gamma_M] = run_stsmpc2(A , B , Ts , Q , R , n_x , n_u , n_m , true_gamma , mW ,  varW , ivarW , mu_gamma_M , var_gamma_M , p_M ,  x ,  T , N , k_fault , delay , ref,  N_s , L , w_ , simulation , sol, t, u);


    toc
end


simulation.x = x;
simulation.u = u;
simulation.p_M = p_M;
% simulation.closedloop_cost = closedloop_cost;
simulation.w_ = w_;
simulation.mu_gamma_M = mu_gamma_M;
simulation.var_gamma_M = var_gamma_M;



subplot(3,1,1)
plot([0:Ts:T*Ts],simulation.x')
title('x','Interpreter','latex')
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


