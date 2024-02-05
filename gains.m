clc; clear; close all;

%%
% import csv files and make them ready for system identification

% load data
a = ['0_5','0_75','1_0','1_25','1_5','2_0','3_0','4_0','5_0'];


% open each csv, create new variable, and save matrix in variable

a1 = readmatrix('./sysiddata/arm_sysid_data4_0_5.csv');
a2 = readmatrix('./sysiddata/arm_sysid_data4_0_75.csv');
a3 = readmatrix('./sysiddata/arm_sysid_data4_1_0.csv');
a4 = readmatrix('./sysiddata/arm_sysid_data4_1_25.csv');
a5 = readmatrix('./sysiddata/arm_sysid_data4_1_5.csv');
a6 = readmatrix('./sysiddata/arm_sysid_data4_2_0.csv');
a7 = readmatrix('./sysiddata/arm_sysid_data4_3_0.csv');
a8 = readmatrix('./sysiddata/arm_sysid_data4_4_0.csv');
a9 = readmatrix('./sysiddata/arm_sysid_data4_5_0.csv');

%%
% remove outliers
t1 = a1(1,:);
v1 = a1(2,:);
c1 = a1(3,:);
[v1,v1la] = rmoutliers(v1);
t1(v1la==1) = [];
c1(v1la==1) = [];

t2 = a2(1,:);
v2 = a2(2,:);
c2 = a2(3,:);
[v2,v2la] = rmoutliers(v2);
t2(v2la==1) = [];
c2(v2la==1) = [];

t3 = a3(1,:);
v3 = a3(2,:);
c3 = a3(3,:);
[v3,v3la] = rmoutliers(v3);
t3(v3la==1) = [];
c3(v3la==1) = [];

t4 = a4(1,:);
v4 = a4(2,:);
c4 = a4(3,:);
[v4,v4la] = rmoutliers(v4);
t4(v4la==1) = [];
c4(v4la==1) = [];

t5 = a5(1,:);
v5 = a5(2,:);
c5 = a5(3,:);
[v5,v5la] = rmoutliers(v5);
t5(v5la==1) = [];
c5(v5la==1) = [];

t6 = a6(1,:);
v6 = a6(2,:);
c6 = a6(3,:);


t7 = a7(1,:);
v7 = a7(2,:);
c7 = a7(3,:);


t8 = a8(1,:);
v8 = a8(2,:);
c8 = a8(3,:);


t9 = a9(1,:);
v9 = a9(2,:);
c9 = a9(3,:);

%%

w = 2*pi*[0.5,0.75,1,1.25,1.5,2,3,4,9];


mm1 = 0.493;
mm2 = 0.492;
mm3 = 0.491;
mm4 = 0.49;
mm5 = 0.4881;
mm6 = 0.4816;
mm7 = 0.4162;
mm8 = 0.3225;
mm9 = 0.2622;

g = [mm1,mm2,mm3,mm4,mm5,mm6,mm7,mm8,mm9];

gg = 20*log10(g);


p = [];
p1 = phdiffmeasure(c1, v1);
p1 = rad2deg(p1);

p2 = phdiffmeasure(c2, v2);
p2 = rad2deg(p2);

p3 = phdiffmeasure(c3, v3);
p3 = rad2deg(p3);

p4 = phdiffmeasure(c4, v4);
p4 = rad2deg(p4);

p5 = phdiffmeasure(c5, v5);
p5 = rad2deg(p5);

p6 = phdiffmeasure(c6, v6);
p6 = rad2deg(p6);

p7 = phdiffmeasure(c7, v7);
p7 = rad2deg(p7);

p8 = phdiffmeasure(c8, v8);
p8 = rad2deg(p8);

p9 = phdiffmeasure(c9, v9);
p9 = rad2deg(p9);

p = [p1,p2,p3,p4,p5,p6,p7,p8,p9];

%%
[mag,phase,wout] = bode(tf(29.08,[1 31.84]));

mag1 = [];
phase2 = [];
for i=1:48
    mag1 = [mag1,mag(1,1,i)];
    phase2 = [phase2,phase(1,1,i)];
end

subplot(2,1,1)
semilogx(w,gg,'.-')

% hold on 
% semilogx(wout,mag1)
% hold off


xlim([1,100])
ylim([-12, 0])
% xlabel('freq(rad/s)')
ylabel('gain(dB)')
title('Bode Gain Plot')
grid on

subplot(2,1,2)
semilogx(w,p,'.-')

% hold on 
% semilogx(wout,phase2)


xlabel('freq(rad/s)')
ylabel('phase(deg)')
% title('Bode Phase Plot')
grid on
xlim([1,100])
ylim([-91,0])

figure(2)
% H = tf(1,[1 9]);
% bode(H)
% xlim([1,100])
% title('1')
% figure(3)
bode(tf(29.08,[1 31.84]))
title('fitted model')
xlim([1,100])

%%

tfa = tf(29.08,[1 31.84]);
ss(tfa)

