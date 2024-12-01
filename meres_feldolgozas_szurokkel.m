fileID=fopen('datav3.txt','r');
meres0 = readmatrix("datav3.txt");
figure('Name', 'Mérési adatok');
[m,n] = size(meres0);
t=0.075;
GyroCovariance=[0.0257,0.0201,0.0254];
giro_drift_cov=[0.2062,0.1011,0.0215];
GyroBias=[0.001 0.0015 -0.00046];
GyroError=[0.4,0.9,0.9];
AbsError=[0.4,0.3,0.3];
time= 1:m;
o_c=0;
o11=0;
o12=0;
o13=0;
o21=0;
o22=0;
o23=0;
o31=0;
o32=0;
o33=0;
o41=0;
o42=0;
o43=0;
o51=0;
o52=0;
o53=0;
o61=0;
o62=0;
o63=0;

for i=1:m
    time(i)=t*i;
end

yaw_abs=1:m;
pitch_abs=1:m;
roll_abs=1:m;
yaw_rel=1:m;
pitch_rel=1:m;
roll_rel=1:m;
yaw_comp=1:m;
pitch_comp=1:m;
roll_comp=1:m;
yaw_kal=1:m;
pitch_kal=1:m;
roll_kal=1:m;
yaw_mad=1:m;
pitch_mad=1:m;
roll_mad=1:m;
yaw_mah=1:m;
pitch_mah=1:m;
roll_mah=1:m;
Euler_Angles_Measured=[m,3];
for i=1:m
Euler_Angles_Measured(i,1:3)=[meres0(i,3),meres0(i,2),meres0(i,1)];
Acc_Z=meres0(i,3);
Acc_Y=meres0(i,2);
Acc_X=meres0(i,1);
Acc_X=Acc_X/26.3;
Acc_Y=Acc_Y/26.3;
Acc_Z=Acc_Z/26.3;
Accelerometer=[Acc_X,Acc_Y,Acc_Z];
Gyro_Z=meres0(i,6);
Gyro_Y=meres0(i,5);
Gyro_X=meres0(i,4);
Gyroscope=[Gyro_X,Gyro_Y,Gyro_Z];
Gyroscope_rad=Gyroscope*pi/180;
Mag_Z=meres0(i,9);
Mag_Y=meres0(i,8);
Mag_X=meres0(i,7);
Mag_X=Mag_X-4.1;
Mag_Y=Mag_Y+28;
Mag_Z=Mag_Z-13;
Magnetometer=[Mag_X,Mag_Y,Mag_Z];
% Accelerometer=[meres0(i,3),meres0(i,2),meres0(i,1)];
% Gyroscope=[meres0(i,6),meres0(i,5),meres0(i,4)];
% Gyroscope_rad=Gyroscope*pi/180;
% Magnetometer=[meres0(i,9),meres0(i,8),meres0(i,7)];
% Accelerometer=Accelerometer/26.3;
% Magnetometer(1)=Magnetometer(1)-4.1;
% Magnetometer(2)=Magnetometer(2)+28;
% Magnetometer(3)=Magnetometer(3)-13;
Q_Pos0 = get_pos('SamplePeriod', 1/14);
Q_Pos0.Get_Pos_Abs(Accelerometer, Magnetometer);
Q_abs = Q_Pos0.Q_Pos_Abs;
if i==1
Q_rel=Q_abs;
end
Q_Pos0.Get_Pos_Rel(Q_rel,Gyroscope);
Q_rel = Q_Pos0.Q_Pos_Rel;
if i==1
 MADG = MadgwickAHRS('SamplePeriod', 1/14,'Quaternion',Q_abs,'Beta', 0.3);
 MAHO = Mahony('SamplePeriod', 1/14,'Quaternion',Q_abs, 'Kp', 1.5,'Ki',0.03);
 COMP = complementary('Alpha',0.1);
 KALM = KalmanFilter('SamplePeriod', 1/14,'GyroCovariance',GyroCovariance ,'GyroBias',GyroBias,'GyroError',GyroError,'AbsError',AbsError);
end
if i==115
    oil=0;
end
COMP.Update(Q_rel,Q_abs);
Q_comp= COMP.Quaternion;
KALM.Update(Gyroscope_rad,Q_abs);
Q_kal= KALM.Quaternion;
MADG.Update(Gyroscope_rad, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mad = MADG.Quaternion;
MAHO.Update(Gyroscope_rad, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mah = MAHO.Quaternion;

% [a,b,c,d]=parts(Q_abs);
% Eul_Angles1=quat2eul([a,b,c,d]); 
%Eul_Angles1=euler(Q_abs,"ZYX","frame");
[a,b,c,d]=parts(Q_abs);
Q_abs1=[a,b,c,d];
Eul_Angles1=quatern2euler(Q_abs);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles1_prev,Eul_Angles1);
o11=o11+k1;
o12=o12+k2;
o13=o13+k3;
end
yaw_abs(i)= Eul_Angles1(1)+(o11*2*pi);
pitch_abs(i)=Eul_Angles1(2)+(o12*2*pi);
roll_abs(i)=Eul_Angles1(3)+(o13*2*pi);
% [a,b,c,d]=parts(Q_rel);
% Eul_Angles2=quat2eul([a,b,c,d]); 
%Eul_Angles2=euler(Q_rel,"ZYX","frame");
[a,b,c,d]=parts(Q_rel);
Q_rel1=[a,b,c,d];
Eul_Angles2=quatern2euler(Q_rel);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles2_prev,Eul_Angles2);
o21=o21+k1;
o22=o22+k2;
o23=o23+k3;
end
yaw_rel(i)= Eul_Angles2(1)+(o21*2*pi);
pitch_rel(i)=Eul_Angles2(2)+(o22*2*pi);
roll_rel(i)=Eul_Angles2(3)+(o23*2*pi);
% [a,b,c,d]=parts(Q_comp);
% Eul_Angles3=quat2eul([a,b,c,d]);  
%Eul_Angles3=euler(Q_comp,"ZYX","frame");
[a,b,c,d]=parts(Q_comp);
Q_comp1=[a,b,c,d];
Eul_Angles3=quatern2euler(Q_comp);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles3_prev,Eul_Angles3);
o31=o31+k1;
o32=o32+k2;
o33=o33+k3;
end
yaw_comp(i)= Eul_Angles3(1)+(o31*2*pi);
pitch_comp(i)=Eul_Angles3(2)+(o32*2*pi);
roll_comp(i)=Eul_Angles3(3)+(o33*2*pi);
% [a,b,c,d]=parts(Q_kal);
% Eul_Angles4=quat2eul([a,b,c,d]); 
%Eul_Angles4=euler(Q_kal,"ZYX","frame");
[a,b,c,d]=parts(Q_kal);
Q_kal1=[a,b,c,d];
Eul_Angles4=quatern2euler(Q_kal);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles4_prev,Eul_Angles4);
o41=o41+k1;
o42=o42+k2;
o43=o43+k3;
end
yaw_kal(i)= Eul_Angles4(1)+(o41*2*pi);
pitch_kal(i)=Eul_Angles4(2)+(o42*2*pi);
roll_kal(i)=Eul_Angles4(3)+(o43*2*pi);
% [a,b,c,d]=parts(Q_mad);
% Eul_Angles5=quat2eul([a,b,c,d]);
%Eul_Angles5=euler(Q_mad,"ZYX","frame");
[a,b,c,d]=parts(Q_mad);
Q_mad1=[a,b,c,d];
Eul_Angles5=quatern2euler(Q_mad);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles5_prev,Eul_Angles5);
o51=o51+k1;
o52=o52+k2;
o53=o53+k3;
end
yaw_mad(i)= Eul_Angles5(1)+(o51*2*pi);
pitch_mad(i)=Eul_Angles5(2)+(o52*2*pi);
roll_mad(i)=Eul_Angles5(3)+(o53*2*pi);
% [a,b,c,d]=parts(Q_mah);
% Eul_Angles6=quat2eul([a,b,c,d]); 
% Eul_Angles6=euler(Q_mah,"ZYX","frame");
[a,b,c,d]=parts(Q_mah);
Q_mah1=[a,b,c,d];
Eul_Angles6=quatern2euler(Q_mah);
if o_c>0
[k1,k2,k3]=overflow_finder(Eul_Angles6_prev,Eul_Angles6);
o61=o61+k1;
o62=o62+k2;
o63=o63+k3;
end
yaw_mah(i)= Eul_Angles6(1)+(o61*2*pi);
pitch_mah(i)=Eul_Angles6(2)+(o62*2*pi);
roll_mah(i)=Eul_Angles6(3)+(o63*2*pi);
Eul_Angles1_prev=Eul_Angles1;
Eul_Angles2_prev=Eul_Angles2;
Eul_Angles3_prev=Eul_Angles3;
Eul_Angles4_prev=Eul_Angles4;
Eul_Angles5_prev=Eul_Angles5;
Eul_Angles6_prev=Eul_Angles6;
o_c=o_c+1;
if i==1 
 figure;
 poseplot
 xlabel("Észak-x (m)")
 ylabel("Kelet-y (m)")
 zlabel("Lent-z (m)")
end
position = [1 1 1];
poseplot(Q_rel,position,"enu")
hold on
position = [3 3 3];
poseplot(Q_abs,position,"enu")
hold on
poseplot(Q_base,[5 5 5],"enu",ScaleFactor=1)
hold on
position = [7 7 7];
poseplot(Q_comp,position,"enu")
hold on
position = [9 9 9];
poseplot(Q_kal,position,"enu")
hold on
position = [11 11 11];
poseplot(Q_mad,position,"enu")
hold on
position = [13 13 13];
poseplot(Q_mah,position,"enu")
pause(0.1);
hold off
end

axis(1) = subplot(3,1,1);
hold on;
%plot(time, Euler_Angles_Measured(1), 'w');
plot(time, yaw_abs, 'r');
plot(time, yaw_rel, 'g');
plot(time, yaw_comp, 'c');
plot(time, yaw_kal, 'm');
plot(time, yaw_mad, 'y');
plot(time, yaw_mah, 'k');
legend('Abszolút mérés','Relatív mérés','Complementary szűrő','Kálmán szűrő','Madgwick szűrő','Mahony szűrő');
xlabel('Idő (s)');
ylabel('yaw (°)');
title('X tengely körüli szögelfordulás');
hold off;
axis(2) = subplot(3,1,2);
hold on;
%plot(time, Euler_Angles_Measured(2), 'w');
plot(time, pitch_abs, 'r');
plot(time, pitch_rel, 'g');
plot(time, pitch_comp, 'c');
plot(time, pitch_kal, 'm');
plot(time, pitch_mad, 'y');
plot(time, pitch_mah, 'k');
legend('Abszolút mérés','Relatív mérés','Complementary szűrő','Kálmán szűrő','Madgwick szűrő','Mahony szűrő');
xlabel('Idő (s)');
ylabel('pitch (°)');
title('Y tengely körüli szögelfordulás');
hold off;
axis(3) = subplot(3,1,3);
hold on;
%plot(time, Euler_Angles_Measured(3), 'w');
plot(time, roll_abs, 'r');
plot(time, roll_rel, 'g');
plot(time, roll_comp, 'c');
plot(time, roll_kal, 'm');
plot(time, roll_mad, 'y');
plot(time, roll_mah, 'k');
legend('Abszolút mérés','Relatív mérés','Complementary szűrő','Kálmán szűrő','Madgwick szűrő','Mahony szűrő');
xlabel('Idő (s)');
ylabel('roll (°)');
title('Z tengely körüli szögelfordulás');
hold off;
linkaxes(axis, 'x');
fclose(fileID);


function [k1,k2,k3]=overflow_finder(Eules_Angles_prev,Eules_Angles)
k1=0;
k2=0;
k3=0;
% if Eules_Angles_prev(1)-Eules_Angles(1)>(4*pi/3)
%     k1=k1+1;
% elseif Eules_Angles(1)-Eules_Angles_prev(1)>(4*pi/3)
%     k1=k1-1;
% end
% if Eules_Angles_prev(2)-Eules_Angles(2)>(4*pi/3)
%     k2=k2+1;
% elseif Eules_Angles(2)-Eules_Angles_prev(2)>(4*pi/3)
%     k2=k2-1;
% end
% if Eules_Angles_prev(3)-Eules_Angles(3)>(4*pi/3)
%     k3=k3+1;
% elseif Eules_Angles(3)-Eules_Angles_prev(3)>(4*pi/3)
%     k3=k3-1;
% end
end

