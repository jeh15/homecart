clc;clear;close all;

M = readmatrix("data/data_movel_2d_2024_01_08-05_45_47_PM.csv");


time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
ball_acc = M(4,:);
target_vel = M(5,:);


%%

pt = -0.5189370547353503;
target = pt*ones(size(time));

subplot(4,1,1)
plot(time,ball_pos,time,target)
title("Ball Position")
xlabel("Time (s)")
ylabel("Position (m)")

subplot(4,1,2)
plot(time,ball_vel)
title("Ball Velocity")
xlabel("Time (s)")
ylabel("Velocity (m/s)")

subplot(4,1,3)
plot(time,ball_acc)
title("Ball Acceleration")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")

subplot(4,1,4)
plot(time,target_vel,time,zeros(size(time)))
title("Target Velocity")
xlabel("Time (s)")
ylabel(" target Velocity (m/s)")

%% MPC
addpath(genpath([pwd '/Functions']))
[Th,Nodes,xd_lb,xd_ub] = DevMPC6();
qd_des = [pt,0,0]';

%% loop plot

xl = [time(1),time(end)+0.5];

ylp = [-1 -.1]; 
% ylp = [-10 100]; 
ylv = [min(ball_vel)-1 max(ball_vel)+1]; 

xlarm = xl;
xlarm(2) = xl(2);
% ylarm = [min(u2,[],"all") max(u2,[],"all")]; 
ylarm = [-10 10]; 

% p_ax = [-0.1 10.1 -.75 -.5];
% v_ax = [-0.1 10.1 -0.2416 0.1679];
% a_ax = [-0.1 10.1 -3.5323 6.7165];
% j_ax = [-0.1 10.1 -141.8087 221.1771];


t_traj = 0:Th/(Nodes-1):Th;

for i=1:size(time,2)
    subplot(2,2,1)
    % subplot(1,2,1)

    % generate x traj
    x0 = [ball_pos(i), ball_vel(i), ball_acc(i)]';
    [x_traj, u_traj] = RunMPC6(Th,Nodes,x0,qd_des,xd_lb,xd_ub);   

    plot(time(1:i),ball_pos(1:i),'.-b',"MarkerSize",12)
    hold on; plot([min(time) max(time)],[ball_pos(1) ball_pos(1)],'--r'); 

    % plot t traj
    plot(time(i) + t_traj,x_traj(1,:)','.-r');

    hold off
    % axis(p_ax)
    xlim(xl)
    ylim(ylp)
    ylabel('Position(m)',"FontSize",16)
    title('Position',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;


    subplot(2,2,3)
    plot(time(1:i),ball_vel(1:i),'.-b',"MarkerSize",12)
    
    hold on    
    plot(time(i) + t_traj,x_traj(2,:)','.-r');
    hold off    

   
    % axis(v_ax)
    xlim(xl)
    ylim(ylv)
    ylabel('Velocity(m/s)',"FontSize",16)
    title('Velocity',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;

    subplot(2,2,[2 4])
    % subplot(1,2,2)
    plot(time(1:i),target_vel(1:i),'.-b',"MarkerSize",12)
    hold on
    plot(time(i)+t_traj,u_traj,'.-r',"MarkerSize",12)
    plot([min(time) max(time)],[0 0],'--r')    
    hold off
    % axis([0-.1 .5+.1 -0.21 .21])
    xlim(xlarm)
    ylim(ylarm+[-0.1,0.1])        
    ylabel('Board Rot Vel(m/s^2)',"FontSize",16)
    title('Control input trajectory - Board Rot Vel',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;

    set(gcf, 'Position', get(0, 'Screensize'));
    pause(.1)
end