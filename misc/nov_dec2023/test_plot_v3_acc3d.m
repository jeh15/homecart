clc
clear
close all

%%
%==========================================================================    
% MPC settings
%==========================================================================    
    Tfinal = 1.0*10;         % Tfinal for simulation
    N      = 11;             % Number of nodes (horizon)
    Th     = 0.1*0.5;            % MPC Time horizon
    Tc     = 0.1;            % Control Time horizon
    tmeasure = 0.0;        % initial time (do not change)
    plot_Th = 0.5;
%==========================================================================    
% System Parameters
%==========================================================================        
    % q - [x dx ddx dddx] - { x - distance along pan(inclined plane) }
    Ac = [0  1  0;...
          0  0  1;...
          0  0  0];
    
    m   = 0.003;            % Mass              [kg]
    R   = 0.02;
    Jz  = .5*m*R^2;    % Moment of inertia [kg.m2]
    g   = 9.81;             % Gravity           [m/s2]
    K = m*g / (m+Jz/R^2);
    Bc = [0;          
          0;
          K];
    Cc = [1 0 0];
    Dc = 0;
    [sysd,G] = c2d(ss(Ac,Bc,Cc,Dc),Th,'zoh');
    sysA = sysd.A;
    sysB = sysd.B;
    params.sysA = sysA;
    params.sysB = sysB;
    params.Th = Th;

%%
M = readmatrix("data/data_movel_2d_2023_12_04-02_34_02_PM.csv");
u2 = readmatrix("data/u3_dec4_1433.csv");

time = M(1,:);
ball_pos = M(2,:);

% control input - rot vel
arm_acc = M(5,:);

ball_vel1 = M(3,:);
ball_acc1 = M(4,:);
ball_jerk1 = 0.0*M(4,:);




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

xl = [time(1),time(end)+0.5];

ylp = [-.7 -.2]; 
% ylp = [-10 100]; 
ylv = [min(ball_vel1) max(ball_vel1)]; 
yla = [min(ball_acc1) max(ball_acc1)]; 
ylj = [min(ball_jerk1) max(ball_jerk1)]; 

xlarm = xl;
xlarm(2) = xl(2);
ylarm = [min(u2,[],"all") max(u2,[],"all")]; 

% p_ax = [-0.1 10.1 -.75 -.5];
% v_ax = [-0.1 10.1 -0.2416 0.1679];
% a_ax = [-0.1 10.1 -3.5323 6.7165];
% j_ax = [-0.1 10.1 -141.8087 221.1771];


t_traj = 0:plot_Th/10:plot_Th;

for i=1:n
    subplot(4,2,1)
    % subplot(1,2,1)

    % generate x traj
    x0 = [ball_pos(i), ball_vel1(i), ball_acc1(i)];
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
    % ylim(ylj)    
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


%==========================================================================
%                           system
%==========================================================================
function y = system(t, x, u, Th, apply_flag, sig, params)
    % apply_flag: 1) noise is applied for real system; 0) no noise for prediction
    A = params.sysA;
    B = params.sysB;
    % K = [0.8151    2.7097   11.8373    3.7545];    
    K = [0,0,0];


    y = A*x'+B*(u(1,1));
    
    if apply_flag == 1
        D = [0 0 0 0;
             0 0 0 0;
             0 0 1 0;
             0 0 0 0];
        w = [0;0;0;0];
        w(1) = normrnd(0,sig);
        w(2) = normrnd(0,sig);
        w(3) = normrnd(0,sig);
        w(4) = normrnd(0,sig);
        y = y + D*w;
    end
    y = y';    
end
%==========================================================================
%                           dynamic
%==========================================================================
function [x, t_intermediate, x_intermediate] = dynamic(system, Th, t0, ...
             x0, u, type, apply_flag, sig, params)
    x = system(t0, x0, u, Th, apply_flag, sig, params);
    x_intermediate = [x0; x];
    t_intermediate = [t0, t0+Th];
end
%==========================================================================
%                           compute open loop solution
%==========================================================================
function x = computeOpenloopSolution(system, N, Th, t0, x0, u, type, sig, params)
    x(1,:) = x0;

    uncertainty_flag = 0;
    for k=1:N
        x(k+1,:) = dynamic(system, Th, t0, x(k,:), u(:,k), type, uncertainty_flag, sig, params);
    end
end

