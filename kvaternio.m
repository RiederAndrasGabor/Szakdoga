device = serialport("COM8",115200);

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
 Acc_X=Read_All(1)*19,62; %*acc_scale=2;
 Acc_Y=Read_All(2)*19,62; %*acc_scale;
 Acc_Z=Read_All(3)*19,62; %*acc_scale;
 Gyro_X=Read_All(4);
 Gyro_Y=Read_All(5);
 Gyro_Z=Read_All(6);
 Mag_X=Read_All(7);
 Mag_Y=Read_All(8);
 Mag_Z=Read_All(9);
 Time = 0.125;
Gyro_X_rad=deg2rad(Gyro_X)*Time;
Gyro_Y_rad=deg2rad(Gyro_Y)*Time; 
Gyro_Z_rad=deg2rad(Gyro_Z)*Time;
Gyro_X=Gyro_X*Time;
Gyro_Y=Gyro_Y*Time;
Gyro_Z=Gyro_Z*Time;
Acc_tetta=atan2(Acc_X,Acc_Y);



 Z_now=[Acc_X Acc_Y Acc_Z];
 Y_now=[Mag_X Mag_Y Mag_Z];
 X_now=cross(Z_now,Y_now);
 Base_XYZ=[1 0 0; 0 1 0; 0 0 1];
 Now_XYZ=[X_now(1) X_now(2) X_now(3);Mag_X Mag_Y Mag_Z; Acc_X Acc_Y Acc_Z];
 Base_XYZ=inv(Base_XYZ);
 Rotmat=Now_XYZ*Base_XYZ;
 Q_Rot_arg=rotm2quat(Rotmat);
 Q_Rot=quaternion(Q_Rot_arg(1),Q_Rot_arg(2),Q_Rot_arg(3),Q_Rot_arg(4));
 Q_Base=quaternion(1,0,0,0);
 Q_abs=quatmultiply(Q_Base,Q_Rot,'ReferenceFrame','enu');


 Q_Base=quaternion(1,0,0,0);
 Q_Gyro= quaternion([Gyro_X Gyro_Y Gyro_Z],"eulerd","XYZ","frame");
 Q_rel= quatmultiply(Q_rel,Q_Gyro,'ReferenceFrame','enu');


if i==60 %i==0
    figure;
 poseplot
 xlabel("Észak-x (m)")
 ylabel("Kelet-y (m)")
 zlabel("Lent-z (m)")
 i=i+1;
end
position = [1 1 1];
poseplot(Q_rel,position,"enu")
hold on
position = [3 3 3];
poseplot(Q_Rot,position,"enu")
hold on
poseplot(Q_Base,[5 5 5],"enu",ScaleFactor=1);
hold off

Eul_Angles1=quat2eul(Q_abs); 
yaw_abs= Eul_Angles1(1);
pitch_abs=Eul_Angles1(2);
roll_abs=Eul_Angles1(3);
Eul_Angles2=quat2eul(Q_rel); 
yaw_rel= Eul_Angles2(1);
pitch_rel=Eul_Angles2(2);
roll_rel=Eul_Angles2(3);

% alpha=0.96;
% pitch_rot=alpha*pitch_abs+(1-alpha)*pitch_rel;
% roll_rot=alpha*roll_abs+(1-alpha)*roll_rel;
% yaw_rot=alpha*yaw_abs+(1-alpha)*yaw_rel;
% Q_rot= eul2quat(pitch_rot,roll_rot,_yaw_rot);
% Q_comp=quatmultiply(Q_comp,Q_rot);
% position = [7 7 7];
% poseplot(Q_comp,position,"enu")
% hold off
 end


