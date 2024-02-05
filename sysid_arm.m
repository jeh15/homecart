clc; clear; close all;

% import csv files and make them ready for system identification

% load data
a = ['0_5','0_75','1_0','1_25','1_5','2_0','3_0','4_0','5_0'];


% open each csv, create new variable, and save matrix in variable

a1 = readmatrix('arm_sysid_data_0_5.csv');
a2 = readmatrix('arm_sysid_data_0_75.csv');
a3 = readmatrix('arm_sysid_data_1_0.csv');
a4 = readmatrix('arm_sysid_data_1_25.csv');
a5 = readmatrix('arm_sysid_data_1_5.csv');
a6 = readmatrix('arm_sysid_data_2_0.csv');
a7 = readmatrix('arm_sysid_data_3_0.csv');
a8 = readmatrix('arm_sysid_data_4_0.csv');
a9 = readmatrix('arm_sysid_data_5_0.csv');



% plot first 2 rows of each matrix
subplot(3,3,1)
plot(a1(1,:),a1(2,:))

subplot(3,3,2)
plot(a2(1,:),a2(2,:))

subplot(3,3,3)
plot(a3(1,:),a3(2,:))

subplot(3,3,4)
plot(a4(1,:),a4(2,:))

subplot(3,3,5)
plot(a5(1,:),a5(2,:))

subplot(3,3,6)
plot(a6(1,:),a6(2,:))

subplot(3,3,7)
plot(a7(1,:),a7(2,:))

subplot(3,3,8)
plot(a8(1,:),a8(2,:))

subplot(3,3,9)
plot(a9(1,:),a9(2,:))


% remove outliers
t1 = a1(1,:);
v1 = a1(2,:);
c1 = a1(3,:);
[v1,v1la] = rmooutliers(v1);
t1(v1la==1) = [];
c1(v1la==1) = [];

t2 = a2(1,:);
v2 = a2(2,:);
c2 = a2(3,:);
[v2,v2la] = rmooutliers(v2);
t2(v2la==1) = [];
c2(v2la==1) = [];

t3 = a3(1,:);
v3 = a3(2,:);
c3 = a3(3,:);
[v3,v3la] = rmooutliers(v3);
t3(v3la==1) = [];
c3(v3la==1) = [];

t4 = a4(1,:);
v4 = a4(2,:);
c4 = a4(3,:);
[v4,v4la] = rmooutliers(v4);
t4(v4la==1) = [];
c4(v4la==1) = [];

t5 = a5(1,:);
v5 = a5(2,:);
c5 = a5(3,:);
[v5,v5la] = rmooutliers(v5);
t5(v5la==1) = [];
c5(v5la==1) = [];

t6 = a6(1,:);
v6 = a6(2,:);
c6 = a6(3,:);
[v6,v6la] = rmooutliers(v6);
t6(v6la==1) = [];
c6(v6la==1) = [];

t7 = a7(1,:);
v7 = a7(2,:);
c7 = a7(3,:);
[v7,v7la] = rmooutliers(v7);
t7(v7la==1) = [];
c7(v7la==1) = [];

t8 = a8(1,:);
v8 = a8(2,:);
c8 = a8(3,:);
[v8,v8la] = rmooutliers(v8);
t8(v8la==1) = [];
c8(v8la==1) = [];

t9 = a9(1,:);
v9 = a9(2,:);
c9 = a9(3,:);
[v9,v9la] = rmooutliers(v9);
t9(v9la==1) = [];
c9(v9la==1) = [];


% plot t,v and t,c
figure
subplot(3,3,1)
plot(t1,v1,'b',t1,c1,'r')

subplot(3,3,2)
plot(t2,v2,'b',t2,c2,'r')

subplot(3,3,3)
plot(t3,v3,'b',t3,c3,'r')

subplot(3,3,4)
plot(t4,v4,'b',t4,c4,'r')

subplot(3,3,5)
plot(t5,v5,'b',t5,c5,'r')

subplot(3,3,6)
plot(t6,v6,'b',t6,c6,'r')

subplot(3,3,7)
plot(t7,v7,'b',t7,c7,'r')

subplot(3,3,8)
plot(t8,v8,'b',t8,c8,'r')

subplot(3,3,9)
plot(t9,v9,'b',t9,c9,'r')

tt = [t1,t2,t3,t4,t5,t6,t7,t8,t9];
vv = [v1,v2,v3,v4,v5,v6,v7,v8,v9];
cc = [c1,c2,c3,c4,c5,c6,c7,c8,c9];

%
plot(t1,abs(v1),'b',t1,c1,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm1 = ginput(1)
hold on
plot(t1,-mm1(2)*ones(size(t1)),'m')
plot(t1,mm1(2)*ones(size(t1)),'m')
hold off

waitforbuttonpress

% do  this for all the data sets
plot(t2,abs(v2),'b',t2,c2,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm2 = ginput(1)
hold on
plot(t2,-mm2(2)*ones(size(t2)),'m')
plot(t2,mm2(2)*ones(size(t2)),'m')
hold off

waitforbuttonpress

plot(t3,abs(v3),'b',t3,c3,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm3 = ginput(1)
hold on
plot(t3,-mm3(2)*ones(size(t3)),'m')
plot(t3,mm3(2)*ones(size(t3)),'m')
hold off

waitforbuttonpress

plot(t4,abs(v4),'b',t4,c4,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm4 = ginput(1)
hold on
plot(t4,-mm4(2)*ones(size(t4)),'m')
plot(t4,mm4(2)*ones(size(t4)),'m')
hold off

waitforbuttonpress

plot(t5,abs(v5),'b',t5,c5,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm5 = ginput(1)
hold on
plot(t5,-mm5(2)*ones(size(t5)),'m')
plot(t5,mm5(2)*ones(size(t5)),'m')
hold off

waitforbuttonpress

plot(t6,abs(v6),'b',t6,c6,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm6 = ginput(1)
hold on
plot(t6,-mm6(2)*ones(size(t6)),'m')
plot(t6,mm6(2)*ones(size(t6)),'m')
hold off

waitforbuttonpress

plot(t7,abs(v7),'b',t7,c7,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm7 = ginput(1)
hold on
plot(t7,-mm7(2)*ones(size(t7)),'m')
plot(t7,mm7(2)*ones(size(t7)),'m')
hold off

waitforbuttonpress

plot(t8,abs(v8),'b',t8,c8,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm8 = ginput(1)
hold on
plot(t8,-mm8(2)*ones(size(t8)),'m')
plot(t8,mm8(2)*ones(size(t8)),'m')
hold off

waitforbuttonpress

plot(t9,abs(v9),'b',t9,c9,'r')
set(gcf, 'Position', get(0, 'Screensize'));
ylim([0.2,0.6])
mm9 = ginput(1)
hold on
plot(t9,-mm9(2)*ones(size(t9)),'m')
plot(t9,mm9(2)*ones(size(t9)),'m')
hold off

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