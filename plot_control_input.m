clc
clear
close all

M = readmatrix("data/data2_2023_10_27-01_23_30_PM.csv");

time = M(1,:);
ball_pos = M(2,:);
arm_acc = M(3,:);

u_traj = readmatrix("data/u1.csv");

%%

N = 11;
Th = 0.1;
t_traj = 0:Th/(N-1):Th;

pos_axlims = [0.4 2.2 -.8 -.3];

acc_axlims = [0-.01 .1+.01 -.02 .02];

for i = 1:size(time,2)
    subplot(1,2,1)
    plot(time(1:i),ball_pos(1:i),'.-b','MarkerSize',12)
    hold on
    plot([-1,3],-.63*ones(1,2),'--r')
    xlabel('time(s)','FontSize',20)
    ylabel('ball position (m)','FontSize',20)
    title('Ball position','FontSize',20)
    axis(pos_axlims)
    ax = gca;
    ax.FontSize = 12;

    subplot(1,2,2)
    plot(t_traj,u_traj(i,:),'.-b','MarkerSize',12)
    xlabel('time(s)','FontSize',20)
    ylabel('Arm acceleration traj (rad/s^2)','FontSize',20)
    title('Arm acceleration traj','FontSize',20)    
    axis(acc_axlims)
    ax = gca;
    ax.FontSize = 12;
    
    pause(1)
end

%%
plot(M(3,:),M(2,:),'.-','MarkerSize',20)
hold on
plot([0,3.5],[0,0],'--')
xlabel('time(s)','FontSize',20)
ylabel('acceleration * dt (rad/s)','FontSize',20)
title('Control input:  joint speed increase (delta V)','FontSize',20)

%%

figure(2)
plot(M(3,:),cumsum(M(2,:)),'.-','MarkerSize',20)
hold on
plot([0,3.5],[0,0],'--')
xlabel('time(s)','FontSize',20)
ylabel('Joint speed (rad/s)','FontSize',20)
title('Joint speed commanded ','FontSize',20)


%% control input array

u_all = readmatrix("u_data.csv");


