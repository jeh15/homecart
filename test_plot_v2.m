clc
clear
close all

M = readmatrix("data/data2_2023_11_01-04_08_28_PM.csv");
u2 = readmatrix("data/u407.csv");

time = M(1,:);
ball_pos = M(2,:);
arm_acc = M(3,:);
ball_vel1 = M(4,:);
ball_acc1 = M(5,:);
ball_jerk1 = M(6,:);




subplot(4,1,1)
plot(time,ball_pos,'.-r')

subplot(4,1,2)
plot(time,ball_vel1,'.-r')

subplot(4,1,3)
plot(time,ball_acc1,'.-r')

subplot(4,1,4)
plot(time,ball_jerk1,'.-r')

n = size(ball_pos,2);

%%

xl = [time(1),time(end)];

ylp = [-.7 -.4]; 
ylv = [min(ball_vel1) max(ball_vel1)]; 
yla = [min(ball_acc1) max(ball_acc1)]; 
ylj = [min(ball_jerk1) max(ball_jerk1)]; 

xlarm = xl;
xlarm(2) = xl(2) + 0.5;
ylarm = [min(u2,[],"all") max(u2,[],"all")]; 

% p_ax = [-0.1 10.1 -.75 -.5];
% v_ax = [-0.1 10.1 -0.2416 0.1679];
% a_ax = [-0.1 10.1 -3.5323 6.7165];
% j_ax = [-0.1 10.1 -141.8087 221.1771];


for i=1:n
    subplot(4,2,1)
    plot(time(1:i),ball_pos(1:i),'.-b',"MarkerSize",12)
    hold on; plot([min(time) max(time)],[ball_pos(1) ball_pos(1)],'--r'); hold off
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
    ylabel('Jerk(m?s^3)',"FontSize",16)
    xlabel('Time(s)',"FontSize",22)
    title('Jerk',"FontSize",16)

    ax = gca;
    ax.FontSize = 16;


    subplot(4,2,[2 4 6 8])
    plot(time(1:i),arm_acc(1:i),'.-b',"MarkerSize",12)
    hold on
    plot(time(i)+[0:.5/10:.5],u2(i,:),'.-r',"MarkerSize",12)
    plot([min(time) max(time)],[0 0],'--r')    
    hold off
    % axis([0-.1 .5+.1 -0.21 .21])
    xlim(xlarm)
    ylim(ylarm)        
    ylabel('Acceleration(m/s^2)',"FontSize",16)
    title('Control input trajectory - Acceleration',"FontSize",22)
    xlabel('Time(s)',"FontSize",16)    

    ax = gca;
    ax.FontSize = 16;

    set(gcf, 'Position', get(0, 'Screensize'));
    pause(.5)
end
