clc; clear; close all

%%

M = readmatrix("data/mpc_1dlearn_trueadapt_2024_09_13-11_07_57_PM.csv");

time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
board_pos = M(4,:);
ball_jerk = M(5,:);
target_vel = M(6,:);
delay = 0.0000001;
kf2_pos = M(7,:);
kf2_vel = M(8,:);
kf2_acc = M(9,:);
k_model = M(10,:);

plot(time,k_model,'.-','LineWidth',1,'MarkerSize',12)
    
% plot([0:Ts:T*Ts],x)
title('Greedy learning - 3 orders of magnitude - K plot','Interpreter','latex')
xlabel('time(s)','Interpreter','latex')
ylabel('${K[\frac{m}{s^2 rad}]}$','interpreter','latex')    
ax = gca;
ax.FontSize = 20;
ax.TickLabelInterpreter = 'latex';
grid on