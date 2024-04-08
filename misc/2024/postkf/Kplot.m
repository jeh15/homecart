clc;clear;close all;

M = readmatrix("data/mpc_1dlearn_2024_02_21-11_33_40_AM.csv");

%%
time = M(1,:);
ball_pos = M(2,:);
ball_vel = M(3,:);
board_pos = M(4,:);
board_vel = M(5,:);
board_targ_vel = M(6,:);
kf2_pos = M(7,:);
kf2_vel = M(8,:);
kf2_acc = M(9,:);
%%
plot(time,ball_pos,time,kf2_pos)
legend('kf1', 'kf2')
title('ball position')

figure(2)
plot(time,ball_vel,time,kf2_vel)
title('ball vel')
legend('kf1', 'kf2')

ba2 = (ball_vel(2:end)-ball_vel(1:end-1))./(time(2:end)-time(1:end-1));

figure(3)
plot(time,kf2_acc,'.-r',time(2:end),(ball_vel(2:end)-ball_vel(1:end-1))./(time(2:end)-time(1:end-1)),'.-b')
title('ball acc')
legend('kf1', 'kf2')

figure(4)
plot(time,board_pos)
title('board position')


figure(5)
%%
plot(time(1:end),kf2_acc./board_pos,'.-',time,0.0*ones(size(time)))
title('ball acc / board pos ==> K')

K_model = kf2_acc./board_pos;
k2=[];

k2min = -17.8;
k2max = 8.8;
for i=1:size(K_model,2)
    if K_model(i)>k2min && K_model(i)<k2max
        k2 = [k2,K_model(i)];
    end
end

figure(6)
histogram(K_model,'NumBins',100)

figure(7)
histogram(k2,'NumBins',100)

