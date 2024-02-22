clc;clear;close all;

M = readmatrix("data/mpc_1dlearn_2024_02_21-11_33_40_AM.csv");
videoname = '/home/orl/Downloads/homecart_misc/videos/kvid123.avi';
numFrames = 331-1;
prefix = '/home/orl/Downloads/vid2pix/t20/frame';

%%
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
%%

pics = {};
for i=0:numFrames    
    filename = strcat(prefix,num2str(i),'.jpg');
    pics{i+1} = imread(filename);
end


%% loop plot

xl = [time(1),time(end)];

ylp = [-100 100]; 
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


% t_traj = 0:Th/(Nodes-1):Th;

figure
set(gcf, 'Position', get(0, 'Screensize'));

pause(.1)

writerObj = VideoWriter(videoname); % Name it.
writerObj.FrameRate = 30; % How many frames per second.
open(writerObj);

K_model = kf2_acc./board_pos;


for i=1:size(time,2)

% ===== k model ====    

    subplot(2,2,1)
    plot(board_pos(1:i),kf2_acc(1:i),'.','MarkerSize',15); grid on
    hold on 
    plot([-.15,.15],[0,0],'--k')
    hold on 
    plot([0,0],[-1,1],'--k')
    % axis equal
    
    title('board th v. ball ddx',"FontSize",22)
    ylabel('ball ddx (m/s)',"FontSize",16)
    xlabel('board th (rad)',"FontSize",16)
    ax = gca;
    ax.FontSize = 16;
    hold off


% ===== k model ====    

    subplot(2,2,3)
    if i>1
        m=fit(board_pos(1:i)',kf2_acc(1:i)','poly1');
        plot(board_pos(1:i),kf2_acc(1:i),'.r')
        hold on 
        plot(board_pos(1:i),m.p1*board_pos(1:i),'-b')
    
        % axis equal
        
        title('plot fitting',"FontSize",22)
        ylabel('ball ddx (m/s)',"FontSize",16)
        xlabel('board th (rad)',"FontSize",16)
        ax = gca;
        ax.FontSize = 16;
        
        text(.1,.4,num2str(m.p1))
        axis([ -.06 .12 -.8 .6 ])
        
        hold off    
    end
% ===== im ====    

    subplot(2,2,[2,4])
    imshow(pics{i})

    set(gcf, 'Position', get(0, 'Screensize'));

    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);    
    
    pause(delay)
end
close(writerObj);

%%

% ball_pos = M(2,:);
% ball_vel = M(3,:);
% board_pos = M(4,:);
% ball_jerk = M(5,:);



figure(1)
plot(time,ball_pos)
ylabel('ball position(m)',"FontSize",16)
title('ball position',"FontSize",22)
xlabel('Time(s)',"FontSize",16)

figure(2)
plot(time,ball_vel)
ylabel(' ball_vel(m/s)',"FontSize",16)
title('ball_vel',"FontSize",22)
xlabel('Time(s)',"FontSize",16)

figure(3)
plot(time,board_pos)
ylabel('board_pos(rad)',"FontSize",16)
title('board_pos',"FontSize",22)
xlabel('Time(s)',"FontSize",16)

figure(4)
plot(time,ball_jerk)
ylabel('board vel (rad/s)',"FontSize",16)
title('board vel',"FontSize",22)
xlabel('Time(s)',"FontSize",16)

%%

plot(board_pos,kf2_acc,'.','MarkerSize',15); grid on
hold on 
plot([-1,1],[0,0],'--k')
hold on 
plot([0,0],[-1,1],'--k')
axis equal

title('board th v. ball ddx',"FontSize",22)
ylabel('ball ddx (m/s)',"FontSize",16)
xlabel('board th (rad)',"FontSize",16)

%%

m=fit(board_pos',kf2_acc','poly1');
plot(board_pos,kf2_acc,'.r')
hold on 
plot(board_pos,-0.1595*board_pos,'-b')
