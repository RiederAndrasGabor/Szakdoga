device = serialport("COM10",115200);
p=1;
Gyro_meas_datas=zeros(1500,3);
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
 Acc_Z=Read_All(1); 
 Acc_Y=Read_All(2); 
 Acc_X=Read_All(3); 
 Gyro_Z=Read_All(4);
 Gyro_Y=Read_All(5);
 Gyro_X=Read_All(6);
 Mag_Z=Read_All(7);
 Mag_Y=Read_All(8);
 Mag_X=Read_All(9);
 Time = 0.075;
 Acc_X=Acc_X/26;
 Acc_Y=Acc_Y/26;
 Acc_Z=Acc_Z/26;
 Mag_X=Mag_X-4,1;
 Mag_Y=Mag_Y+28;
 Mag_Z=Mag_Z-13;
Gyro_X_rad=deg2rad(Gyro_X)*Time;
Gyro_Y_rad=deg2rad(Gyro_Y)*Time; 
Gyro_Z_rad=deg2rad(Gyro_Z)*Time;
Gyro_X=Gyro_X*Time;
Gyro_Y=Gyro_Y*Time;
Gyro_Z=Gyro_Z*Time;
Acc_tetta=atan2(Acc_X,Acc_Y);



 Z_now=[-Acc_X Acc_Y -Acc_Z];
 Y_now=[Mag_X Mag_Y Mag_Z];
 X_now=cross(Z_now,Y_now);
 Y_now2=cross(X_now,Z_now);
 Base_XYZ=[1 0 0; 0 1 0; 0 0 1];
 Now_XYZ=[X_now(1) X_now(2) X_now(3);Y_now2(1) Y_now2(2) Y_now2(3); Acc_X Acc_Y Acc_Z];
 %Base_XYZ=inv(Base_XYZ);
 Rotmat=Now_XYZ*Base_XYZ;
 Q_Rot_arg=rotm2quat(Rotmat);
 Q_Rot=quaternion(Q_Rot_arg(1),Q_Rot_arg(2),Q_Rot_arg(3),Q_Rot_arg(4));
 Q_Rot=quatinv(Q_Rot);
 Q_Base=quaternion(1,0,0,0);
 Q_abs=quatmultiply(Q_Base,Q_Rot,'ReferenceFrame','enu');
 if i==60 
    Q_rel=Q_abs;
 end

 Q_Gyro= quaternion([-Gyro_X Gyro_Y Gyro_Z],"eulerd","XYZ","frame");
 Q_rel= quatmultiply(Q_rel,Q_Gyro,'ReferenceFrame','enu');
 gyro_angles=quat2eul(Q_rel);
 Gyro_meas_datas(p,1)=gyro_angles(1);
 Gyro_meas_datas(p,2)=gyro_angles(2);
 Gyro_meas_datas(p,3)=gyro_angles(3);
 p=p+1;

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
poseplot(Q_abs,position,"enu")
hold on
poseplot(Q_Base,[5 5 5],"enu",ScaleFactor=1);
hold on
%hold off

Eul_Angles1=quat2eul(Q_abs); 
yaw_abs= Eul_Angles1(1);
pitch_abs=Eul_Angles1(2);
roll_abs=Eul_Angles1(3);
Eul_Angles2=quat2eul(Q_rel); 
yaw_rel= Eul_Angles2(1);
pitch_rel=Eul_Angles2(2);
roll_rel=Eul_Angles2(3);

if i==61
Q_comp=Q_abs;
 i=i+1;
end
alpha=0.9;
pitch_rot=alpha*pitch_abs+(1-alpha)*pitch_rel;
roll_rot=alpha*roll_abs+(1-alpha)*roll_rel;
yaw_rot=alpha*yaw_abs+(1-alpha)*yaw_rel;
eul=[-yaw_rot -pitch_rot -roll_rot];
Q_rot= eul2quat(eul);
Q_comprot= quaternion(Q_rot(1),Q_rot(2),Q_rot(3),Q_rot(4));
Q_comp=quatmultiply(Q_comp,Q_comprot,'ReferenceFrame','enu');
Q_comprot=quatinv(Q_comprot);
position = [7 7 7];
poseplot(Q_comprot,position,"enu")
%pause(0.0001);
hold off
 end





