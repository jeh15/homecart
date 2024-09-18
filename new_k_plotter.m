clc; clear; close all

%%

M = readmatrix("data/mpc_1dlearn_trueadapt_2024_05_07-09_39_59_PM.csv");
videoname = '/home/orl/Downloads/homecart_misc/videos/kvid_may7_3.avi';
numFrames = 68-1;
prefix = '/home/orl/Downloads/vid2pix/k3/frame';

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
k_model = M(10,:);

%%

pics = {};
for i=0:numFrames    
    filename = strcat(prefix,num2str(i),'.jpg');
    pics{i+1} = imread(filename);
end

%%

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

% K_model = kf2_acc./board_pos;


for i=1:numFrames

% ===== k model ====    

    subplot(1,2,1)
    plot(time(1:i),k_model(1:i),'.-','MarkerSize',15); grid on
    hold on
    plot([1.6746,time(i)],[5.886,5.886],'.-','MarkerSize',15); grid on
    % hold on
    % text(time(i)-0.25,5.886+1,'real')
    hold on
    plot([1.6746,time(i)],[0,0],'--k','MarkerSize',15); grid on
    if i>10
        ylim([-80,80])
    end
    legend('Learned Model','Actual Model')
    % hold on 
    % plot([-.15,.15],[0,0],'--k')
    % hold on 
    % plot([0,0],[-1,1],'--k')
    % axis equal
    
    title('Model learning',"FontSize",22)
    ylabel('${K[\frac{m}{s^2 rad}]}$','interpreter','latex','Rotation',0,'FontSize',16)    
    % ylabel('',"FontSize",16)
    xlabel('time(s)','interpreter','latex','Rotation',0,'FontSize',16)
    ax = gca;
    ax.FontSize = 16;
    hold off


% ===== k model ====    

    % subplot(2,2,3)
    % if i>1
    %     % m=fit(board_pos(1:i)',kf2_acc(1:i)','poly1');
    %     plot(board_pos(1:i),kf2_acc(1:i),'.r')
    %     hold on 
    %     plot(board_pos,k_model(i)*board_pos,'-b')
    % 
    %     % axis equal
    % 
    %     title('plot fitting',"FontSize",22)
    %     ylabel('ball ddx (m/s)',"FontSize",16)
    %     xlabel('board th (rad)',"FontSize",16)
    %     ax = gca;
    %     ax.FontSize = 16;
    % 
    %     % text(.1,.4,num2str(k_model))
    %     axis([ -.06 .12 -.8 .6 ])
    % 
    %     hold off    
    % end
% ===== im ====    

    subplot(1,2,2)
    imshow(pics{i})

    set(gcf, 'Position', get(0, 'Screensize'));

    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);    
    
    pause(delay)
end
close(writerObj);