clc
clear
close all


M = readmatrix("data/data_movel_2d_2023_11_30-02_29_42_PM.csv");

% time_data,  ball_pos_data, ball_vel_data, arm_angle_array_data, target_angle_data

time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
arm_angle = M(4,:);
target_angle = M(5,:);
% target_vel = M(6,:);


board_start = [-0.75, 0.0];

board_length = 0.5;

for i=1:12
    
    % actual board
    th = arm_angle(i);
    board_end = [board_start(1)+(board_length*cos(th)), board_start(2)+(board_length*sin(th))];
    boardx = [board_start(1),board_end(1)];
    boardy = [board_start(2),board_end(2)];
    plot(boardx,boardy,'m','LineWidth',1)
    
    hold on


    % target board
    th = -target_angle(i);
    board_end = [board_start(1)+(board_length*cos(th)), board_start(2)+(board_length*sin(th))];
    boardx = [board_start(1),board_end(1)];
    boardy = [board_start(2),board_end(2)];
    plot(boardx,boardy,'--r')
    
    hold on
    


    % ball
    plot(ball_pos(i),0,'or')
    
    axis([-1,0,-0.5,0.5])
    axis equal


    hold on

    % ball target
    plot(ball_pos(1),0,'.b')
    

    pause(0.5)
    clf

end


