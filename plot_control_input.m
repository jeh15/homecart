clc
clear
close all

M = readmatrix("data/data2_2023_10_29-05_21_43_PM.csv");

time = M(1,:);
ball_pos = M(2,:);
arm_acc = M(3,:);
ball_vel1 = M(4,:);
ball_acc1 = M(5,:);
ball_jerk1 = M(6,:);

ball_vel = M(4,:);
ball_acc = M(5,:);
ball_jerk = M(6,:);

u_traj = readmatrix("data/u2.csv");

for i = 7:size(time,2)
    if i > 7
       ball_vel(i) = mean(ball_vel1(i-5:i));
       ball_acc(i) = mean(ball_acc1(i-5:i));
       ball_jerk(i) = mean(ball_jerk1(i-5:i));
    end
   
end

%% plot states

subplot(4,1,1)
plot(time,ball_pos,'.-')

subplot(4,1,2)
plot(time,ball_vel,'.-b',time,ball_vel1,'.-r')

subplot(4,1,3)
plot(time,ball_acc,'.-b',time,ball_acc1,'.-r')

subplot(4,1,4)
plot(time,ball_jerk,'.-b',time,ball_jerk1,'.-r')

figure(2)
plot(time,arm_acc,'.-')

%%

N = 11;
Th = 0.5;
t_traj = 0:Th/(N-1):Th;

% pos_axlims = [0.4 2.2 -.8 -.3];
% 
% acc_axlims = [0-.01 .1+.01 -.02 .02];

for i = 1:size(time,2)
    subplot(1,2,1)
    plot(time(1:i),ball_pos(1:i),'.-b','MarkerSize',12)
    hold on
    plot([-1,3],-.58*ones(1,2),'--r')
    xlabel('time(s)','FontSize',20)
    ylabel('ball position (m)','FontSize',20)
    title('Ball position','FontSize',20)
    % axis(pos_axlims)
    ax = gca;
    ax.FontSize = 12;

    subplot(1,2,2)
    plot(t_traj,u_traj(i,:),'.-b','MarkerSize',12)
    xlabel('time(s)','FontSize',20)
    ylabel('Arm acceleration traj (rad/s^2)','FontSize',20)
    title('Arm acceleration traj','FontSize',20)    
    % axis(acc_axlims)
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


