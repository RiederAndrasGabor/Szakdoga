device = serialport("COM8",115200);


for i=1:60

 sensordata = readline(device);
end

 while 1
 sensordata = readline(device);
 Readings = strrep(sensordata,',',' ');
 Read_All= str2num(Readings);
 Acc_X=Read_All(1);
 Acc_Y=Read_All(2);
 Acc_Z=Read_All(3);
 Gyro_X=Read_All(4);
 Gyro_Y=Read_All(5);
 Gyro_Z=Read_All(6);
 Mag_X=Read_All(7);
 Mag_Y=Read_All(8);
 Mag_Z=Read_All(9);
 Time = 0.01;
Gyro_X_rad=deg2rad(Gyro_X)*Time;
Gyro_Y_rad=deg2rad(Gyro_Y)*Time; 
Gyro_Z_rad=deg2rad(Gyro_Z)*Time;
Gyro_X=Gyro_X*Time;
Gyro_Y=Gyro_Y*Time;
Gyro_Z=Gyro_Z*Time;
Acc_tetta=atan2(Acc_X,Acc_Y);
Q_Acc=quaternion(0,Acc_X,Acc_Y,Acc_Z);


Q_Gyro= quaternion([Gyro_X Gyro_Y Gyro_Z],"eulerd","ZYX","frame");

Q_Acc= quatmultiply(Q_Acc,Q_Gyro);

Acc_Data=Read_All(1:3);
Gyro_Data=Read_All(4:6);
Mag_Data=Read_All(7:9);
Q_Mag= quaternion([Mag_X Mag_Y Mag_Z],"eulerd","ZYX","frame");
Q_Rot =ecompass(Acc_Data,Mag_Data);
Q_Acc= quatmultiply(Q_Acc,Q_Gyro);
Q_Base=quaternion(1,1,1,1);

Q_AccMag=quatmultiply(Q_Base,Q_Rot);
if i==60
 poseplot
 xlabel("Észak-x (m)")
 ylabel("Kelet-y (m)")
 zlabel("Lent-z (m)")
 i=i+1;
end
position = [3 3 3];
poseplot(Q_AccMag,position)
hold on
position = [1 1 1];
poseplot(Q_Acc,position)
hold on
Q_Base2=quaternion(1,1,1,1);
poseplot(Q_Base2,[5 5 5],ScaleFactor=1);
hold off

 end

