clear;
device = serialport("COM10",115200);

for i=1:60
 %acc_scale=2;
 sensordata = readline(device);
end

Q_rel=quaternion(1,0,0,0);
Q_base=quaternion(1,0,0,0);
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
Gyroscope=[Gyro_X,Gyro_Y,Gyro_Z];
Mag_Z=Read_All(9);
Mag_Y=Read_All(8);
Mag_X=Read_All(7);
Time = 0.07143;
Acc_X=Acc_X/26;
Acc_Y=Acc_Y/26;
Acc_Z=Acc_Z/26;
Accelerometer=[Acc_X,Acc_Y,Acc_Z];
Mag_X=Mag_X-4,1;
Mag_Y=Mag_Y+28;
Mag_Z=Mag_Z-13;
Magnetometer=[Mag_X,Mag_Y,Mag_Z];
Acc_tetta=atan2(Acc_X,Acc_Y);

Q_Pos0 = get_pos('SamplePeriod', 1/14);
Q_Pos0.Get_Pos_Abs(Accelerometer, Magnetometer);
Q_abs = Q_Pos0.Q_Pos_Abs;
if i==60 
 figure;
 poseplot
 xlabel("Észak-x (m)")
 ylabel("Kelet-y (m)")
 zlabel("Lent-z (m)")
 i=i+1;
 Q_rel=Q_abs;
end
Q_Pos0.Get_Pos_Rel(Q_rel,Gyroscope);
Q_rel = Q_Pos0.Q_Pos_Rel;

Accelerometer(1)=Acc_X;
Accelerometer(2)=Acc_Y;
Accelerometer(3)=Acc_Z;
Gyroscope(1)=Gyro_X*pi/180;
Gyroscope(2)=Gyro_Y*pi/180;
Gyroscope(3)=Gyro_Z*pi/180;
Magnetometer(1)=Mag_X;
Magnetometer(2)=Mag_Z;
Magnetometer(3)=-Mag_Y;
if i==61
 Q_mad=Q_abs;
 %[Q_mad0,Q_mad1,Q_mad2,Q_mad3] = parts(Q_mad);
 %Q_m=[Q_mad0,Q_mad1,Q_mad2,Q_mad3];
 MADG = MadgwickAHRS('SamplePeriod', 1/14,'Quaternion',Q_mad, 'Beta', 0.3);
 MAHO = Mahony('SamplePeriod', 1/14,'Quaternion',Q_mad, 'Kp', 1.5,'Ki',0.02);
 i=0;
end
MADG.Update(Gyroscope, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mad = MADG.Quaternion;
MAHO.Update(Gyroscope, Accelerometer, Magnetometer);	% gyroscope units must be radians
Q_mah = MAHO.Quaternion;

position = [1 1 1];
poseplot(Q_rel,position,"enu")
hold on
position = [3 3 3];
poseplot(Q_abs,position,"enu")
hold on
poseplot(Q_base,[5 5 5],"enu",ScaleFactor=1)
hold on
position = [11 11 11];
poseplot(Q_mad,position,"enu")
hold on
position = [13 13 13];
poseplot(Q_mah,position,"enu")
pause(0.0001);
hold off

end



% Filter1=complementary
% Filter1.alpha=0.98
% while(1)
%     %quat rot és acc+magos pozíció --> euler szögekké alakítva
%     Filter1.getrot(Q_actual,pitch_rot,roll_rot,yaw_rot,pitch_rel,roll_rel,yaw_rel);
% end