clc
clear
close all

M = readmatrix("data/data_movel_2d_2023_11_29-07_44_12_PM.csv");

% time_data,  ball_pos_data, ball_vel_data, arm_angle_array_data, target_angle_data

time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
arm_angle = M(4,:);
target_angle = M(5,:);


% subplot(3,1,1)
% plot(time,ball_pos,'.-r')
% title('ball position')
% 
% subplot(3,1,2)
% plot(time,ball_vel,'.-r')
% title('ball vel')
% 
% subplot(3,1,3)
% plot(time,arm_angle,'.-r')
% title('arm angle')
% 
% hold on
% plot(time,target_angle,'.-b')
% title('target angle')
% legend('arm','target')


n = 19;


for i=1:n
    subplot(4,2,1)
    % subplot(1,2,1)

    % generate x traj
    x0 = [ball_pos(i), ball_vel1(i)];
    x_traj = computeOpenloopSolution(@system, N, Th, 0, x0, u2(i,:), 0, 0, params);

    plot(time(1:i),ball_pos(1:i),'.-b',"MarkerSize",12)
    hold on; plot([min(time) max(time)],[ball_pos(1) ball_pos(1)],'--r'); 

    % plot t traj
    plot(time(i) + t_traj,x_traj(2:end,1)','.-r');

    hold off
    % axis(p_ax)
    xlim(xl)
    ylim(ylp)
    ylabel('Position(m)',"FontSize",16)
    title('Position',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,3)
    plot(time(1:i),ball_vel1(1:i),'.-b',"MarkerSize",12)
    % axis(v_ax)
    xlim(xl)
    ylim(ylv)
    ylabel('Velocity(m/s)',"FontSize",16)
    title('Velocity',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,5)
    plot(time(1:i),ball_acc1(1:i),'.-b',"MarkerSize",12)
    % axis(a_ax)
    xlim(xl)
    ylim(yla)    
    ylabel('Acceleration(m/s^2)',"FontSize",16)
    title('Acceleration',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,7)
    plot(time(1:i),ball_jerk1(1:i),'.-b',"MarkerSize",12)
    % axis(j_ax)
    xlim(xl)
    ylim(ylj)    
    ylabel('Jerk(m/s^3)',"FontSize",16)
    xlabel('Time(s)',"FontSize",22)
    title('Jerk',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,[2 4 6 8])
    % subplot(1,2,2)
    plot(time(1:i),arm_acc(1:i),'.-b',"MarkerSize",12)
    hold on
    plot(time(i)+[0:.5/10:.5],u2(i,:),'.-r',"MarkerSize",12)
    plot([min(time) max(time)],[0 0],'--r')    
    hold off
    % axis([0-.1 .5+.1 -0.21 .21])
    xlim(xlarm)
    ylim(ylarm+[-0.1,0.1])        
    ylabel('Acceleration(m/s^2)',"FontSize",16)
    title('Control input trajectory - Acceleration',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;

    set(gcf, 'Position', get(0, 'Screensize'));
    pause(.5)
end