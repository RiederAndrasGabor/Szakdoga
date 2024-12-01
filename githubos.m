clear;

device = serialport("COM10",115200);

for i=1:60
 %acc_scale=2;
 sensordata = readline(device);
end
%i=0;
Q_rel=quaternion(1,0,0,0);
 while 1

  % sensordata = readline(device);
  % pat=lettersPattern
  %true_false=contains(sensordata,pat)
  % if true_false=true
  %         continue
  % end
 sensordata = readline(device);
 Readings = strrep(sensordata,',',' ');
 Read_All= str2num(Readings);
 Acc_Z=Read_All(3); 
 Acc_Y=Read_All(2); 
 Acc_X=Read_All(1); 
 Gyro_Z=Read_All(6);
 Gyro_Y=Read_All(5);
 Gyro_X=Read_All(4);
 Gyro_Z1=Read_All(6);
 Gyro_Y1=Read_All(5);
 Gyro_X1=Read_All(4);
 Mag_Z=Read_All(9);
 Mag_Y=Read_All(8);
 Mag_X=Read_All(7);
 Time = 0.07143;
 Acc_X=Acc_X/26;
 Acc_Y=Acc_Y/26;
 Acc_Z=Acc_Z/26;
 Mag_X=Mag_X-4,1;
 Mag_Y=Mag_Y+28;
 Mag_Z=Mag_Z-13;
Gyro_X_rad=deg2rad(Gyro_X);
Gyro_Y_rad=deg2rad(Gyro_Y); 
Gyro_Z_rad=deg2rad(Gyro_Z);
Gyro_X=Gyro_X*Time;
Gyro_Y=Gyro_Y*Time;
Gyro_Z=Gyro_Z*Time;
Acc_tetta=atan2(Acc_X,Acc_Y);



 Z_now=[Acc_X Acc_Y Acc_Z];
 Z_now = Z_now/norm(Z_now);
 Y_now=[Mag_X Mag_Y Mag_Z];
 Y_now = Y_now/norm(Y_now);
 X_now=cross(Y_now,Z_now);
 Y_now2=cross(Z_now,X_now);
 Now_XYZ=[X_now', Y_now2', Z_now'];
 Rotmat=Now_XYZ^-1;
 Q_Rot_arg=rotm2quat(Rotmat);
 Q_Rot=quaternion(Q_Rot_arg(1),Q_Rot_arg(2),Q_Rot_arg(3),Q_Rot_arg(4));
 % Q_Rot=quatinv(Q_Rot);
 Q_Base=quaternion(1,0,0,0);
 Q_abs=quatmultiply(Q_Base,Q_Rot,'ReferenceFrame','enu');
 if i==60 
    Q_rel=Q_abs;
 end

 Q_Gyro= quaternion([Gyro_X Gyro_Y Gyro_Z],"eulerd","XYZ","frame");
 Q_rel= quatmultiply(Q_rel,Q_Gyro,'ReferenceFrame','enu');


if i==60 %i==0
    figure;
 poseplot
 xlabel("Észak-x (m)")
 ylabel("Kelet-y (m)")
 zlabel("Lent-z (m)")
 i=i+1;
 Q_comp=Q_abs;
end
position = [1 1 1];
poseplot(Q_rel,position,"enu")
hold on
position = [3 3 3];
poseplot(Q_Rot,position,"enu")
hold on
poseplot(Q_Base,[5 5 5],"enu",ScaleFactor=1);
hold on
%hold off

Eul_Angles1=quat2eul(Q_Rot); 
yaw_abs= Eul_Angles1(1);
pitch_abs=Eul_Angles1(2);
roll_abs=Eul_Angles1(3);
Eul_Angles2=quat2eul(Q_rel); 
yaw_rel= Eul_Angles2(1);
pitch_rel=Eul_Angles2(2);
roll_rel=Eul_Angles2(3);

time=Time;
giro_bias=0;
giro_cov=[0.0257,0.0201,0.0254];
giro_drift_bias=0;
giro_drift_cov=[0.2062,0.1011,0.0215];
bias_t_prev=[0.001 0.0015 -0.00046];
meresi_hibak=[0,0,0];
yaw_now=yaw_abs;
roll_now=roll_abs;
pitch_now=pitch_abs;
yaw_covariance_current=giro_cov(1);
roll_covariance_current=giro_cov(2);
pitch_covariance_current=giro_cov(3);

if i~=61
 yaw_now=yaw_measured;
 roll_now=roll_measured;
 pitch_now=pitch_measured;
 yaw_covariance_current=yaw_covariance_measured;
 roll_covariance_current=roll_covariance_measured; 
 pitch_covariance_current=pitch_covariance_measured; 
end
%mérés predikció
yaw_pred=yaw_now+time*(yaw_rel-bias_t_prev(1));
roll_pred=roll_now+time*(roll_rel-bias_t_prev(2));
pitch_pred=pitch_now+time*(pitch_rel-bias_t_prev(3));


%kovariancia predikció, 1^2=giroszkóp eltérése
yaw_covariance_predicted=yaw_covariance_current+(time^2*0.9^2);
roll_covariance_predicted=roll_covariance_current+(time^2*0.9^2);
pitch_covariance_predicted=pitch_covariance_current+(time^2*0.9^2);


%Kálmán erősítés számítása; kb 0.3 FOK HIBÁJA LEHET KB AZ ABSZ POZíCIÓ MÉRÉSNEK
kalman_gain_yaw=yaw_covariance_predicted/(yaw_covariance_predicted+0.3^2);
kalman_gain_roll=roll_covariance_predicted/(roll_covariance_predicted+0.3^2);
kalman_gain_pitch=pitch_covariance_predicted/(pitch_covariance_predicted+0.3^2);

% A számított orientáció
yaw_measured=yaw_pred+kalman_gain_yaw*(yaw_abs-yaw_pred);
roll_measured=roll_pred+kalman_gain_roll*(roll_abs-roll_pred);
pitch_measured=pitch_pred+kalman_gain_pitch*(pitch_abs-pitch_pred);


% A számított kovariancia
yaw_covariance_measured= (1-kalman_gain_yaw)*yaw_covariance_predicted;
pitch_covariance_measured= (1-kalman_gain_pitch)*pitch_covariance_predicted;
roll_covariance_measured= (1-kalman_gain_roll)*roll_covariance_predicted;


eul=[yaw_measured pitch_measured  roll_measured];
Q_rot= eul2quat(eul);
Q_comprot= quaternion(Q_rot(1),Q_rot(2),Q_rot(3),Q_rot(4));
% Q_comp=quatmultiply(Q_comp,Q_comprot,'ReferenceFrame','enu');
% Q_comprot=quatinv(Q_comprot);
position = [7 7 7];
poseplot(Q_comprot,position,"enu")
%pause(0.0001);
hold on

yaw_now=yaw_abs;
roll_now=roll_abs;
pitch_now=pitch_abs;

if i~=61
    yaw_now=yaw_rot; 
    pitch_now=pitch_rot; 
    roll_now=roll_rot;
end
alpha=0.1;


%bias_t_prev=[0.001 0.0015 -0.00046];
% pitch_rot=alpha*pitch_abs+(1-alpha)*(pitch_now+(time*Gyro_X));
% roll_rot=alpha*roll_abs+(1-alpha)*(roll_now+(time*Gyro_Y));
% yaw_rot=alpha*yaw_abs+(1-alpha)*(yaw_now+(time*Gyro_Z));
pitch_rot=alpha*pitch_abs+(1-alpha)*pitch_rel;
roll_rot=alpha*roll_abs+(1-alpha)*roll_rel;
yaw_rot=alpha*yaw_abs+(1-alpha)*yaw_rel;
eul=[yaw_rot pitch_rot roll_rot];
Q_rot= eul2quat(eul);
Q_comprot= quaternion(Q_rot(1),Q_rot(2),Q_rot(3),Q_rot(4));
position = [9 9 9];
poseplot(Q_comprot,position,"enu")
hold on
Accelerometer(1)=Acc_X;
Accelerometer(2)=Acc_Y;
Accelerometer(3)=Acc_Z;
Gyroscope(1)=Gyro_X_rad;
Gyroscope(2)=Gyro_Y_rad;
Gyroscope(3)=Gyro_Z_rad;
Magnetometer(1)=Mag_X;
Magnetometer(2)=Mag_Z;
Magnetometer(3)=-Mag_Y;
if i==61
 Q_mad=Q_abs;
 MADG = MadgwickAHRS('SamplePeriod', 1/14,'Quaternion',Q_mad, 'Beta', 0.3);
 MAHO = Mahony('SamplePeriod', 1/14,'Quaternion',Q_mad, 'Kp', 0.9,'Ki',0.01);
end
MADG.Update(Gyroscope, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mad = MADG.Quaternion;
MAHO.Update(Gyroscope, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mah = MAHO.Quaternion;
position = [11 11 11];
poseplot(Q_mad,position,"enu")
hold on
position = [13 13 13];
poseplot(Q_mah,position,"enu")
pause(0.0001);
hold off
i=i+1;
end
