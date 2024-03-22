clc;clear;close all;

M = readmatrix("data/nmpcv2_2024_03_20-07_42_37_PM.csv");
videoname = '/home/orl/Downloads/homecart_misc/videos/out_03_20_2024_1945.avi';
numFrames = 250-1;
prefix = '/home/orl/Downloads/vid2pix/tx4/frame';
pt = -0.0034770493516508333;

Th = 1/13;

Nodes = 10;
%%
time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
ball_acc = M(4,:);
ball_jerk = M(5,:);
target_vel = M(6,:);
delay = 0.0000001;

%%

pics = {};
for i=0:numFrames    
    filename = strcat(prefix,num2str(i),'.jpg');
    pics{i+1} = imread(filename);
end


%%

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
plot(time,ball_jerk,time,zeros(size(time)))
title("Target Velocity")
xlabel("Time (s)")
ylabel(" target Velocity (m/s)")

%% MPC
% addpath(genpath([pwd '/JW_MPC_ball_board_model/Functions']))
% addpath(genpath([pwd '/JW_MPC_ball_board_model']))
% [Th,Nodes,xd_lb,xd_ub] = DevMPC6();
% qd_des = [pt,0,0,0]';

%% loop plot

xl = [time(1),time(end)+0.5];

ylp = [-.3 .3]; 
% ylp = [-10 100]; 
ylv = [min(ball_vel)-1 max(ball_vel)+1]; 
ylv = [-0.3 0.3];
xlarm = xl;
xlarm(2) = xl(2);
% ylarm = [min(u2,[],"all") max(u2,[],"all")]; 
ylarm = [-10 10]; 

% p_ax = [-0.1 10.1 -.75 -.5];
% v_ax = [-0.1 10.1 -0.2416 0.1679];
% a_ax = [-0.1 10.1 -3.5323 6.7165];
% j_ax = [-0.1 10.1 -141.8087 221.1771];

ylbv = [-2 2]; 


t_traj = 0:Th/(Nodes):Th;
t_traj2 = 0:Th/(Nodes-1):Th;

figure
set(gcf, 'Position', get(0, 'Screensize'));

pause(.1)

writerObj = VideoWriter(videoname); % Name it.
writerObj.FrameRate = 29; % How many frames per second.
open(writerObj);


for i=1:numFrames+1
    % i=83;
% ===== ball pos ====    
    subplot(4,2,1)
    % subplot(1,2,1)

    % generate x traj
    x0 = [ball_pos(i), ball_vel(i), ball_acc(i), ball_jerk(i)]';

    [x_traj,u_traj] = nmpc_new(ball_pos(i), ball_vel(i), ball_acc(i), ball_jerk(i),pt);
    
    % x_traj = out(:,1:end-1);
    % u_traj = out(:,end);

    % [x_traj, u_traj] = RunMPC6(Th,Nodes,x0,qd_des,xd_lb,xd_ub);   

    plot(time(1:i),ball_pos(1:i),'.-b',"MarkerSize",12)
    hold on; plot([min(time) max(time)],[ball_pos(1) ball_pos(1)],'--r'); 

    % plot t traj
    plot(time(i) + t_traj,x_traj(:,1),'.-r');

    hold off
    % axis(p_ax)
    xlim(xl)
    ylim(ylp)
    ylabel('Position(m)',"FontSize",16)
    title('Position',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;

% ===== ball vel====    

    subplot(4,2,3)
    plot(time(1:i),ball_vel(1:i),'.-b',"MarkerSize",12)
    
    hold on    
    plot(time(i) + t_traj,x_traj(:,2)','.-r');
    hold on; plot([min(time) max(time)],[0 0],'--r'); 
    
    hold off    

   
    % axis(v_ax)
    xlim(xl)
    ylim(ylv)
    ylabel('Velocity(m/s)',"FontSize",16)
    title('Velocity',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;


% ===== board angle ====    


    subplot(4,2,5)
    plot(time(1:i),ball_acc(1:i),'.-b',"MarkerSize",12)
    
    hold on    
    plot(time(i) + t_traj,x_traj(:,3)','.-r');
    hold off    

   
    % axis(v_ax)
    xlim(xl)
    ylim(ylv)
    ylabel('board pos (rad)',"FontSize",16)
    title('Board position',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;

% ===== board w ====    

    subplot(4,2,7)
    plot(time(1:i),ball_jerk(1:i),'.-b',"MarkerSize",12)
    
    hold on    
    plot(time(i) + t_traj,x_traj(:,4)','.-r');
    hold off    

   
    % axis(v_ax)
    xlim(xl)
    ylim(ylbv)
    ylabel('board ang Velocity(rad/s)',"FontSize",16)
    title('Board Velocity',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,[2 4])
    % subplot(1,2,2)
    plot(time(1:i),target_vel(1:i),'.-b',"MarkerSize",12)
    hold on
    plot(time(i)+t_traj2,u_traj,'.-r',"MarkerSize",12)
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

    subplot(4,2,[6 8])
    imshow(pics{i})


    set(gcf, 'Position', get(0, 'Screensize'));


    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);    
    

    pause(delay)
end
close(writerObj);