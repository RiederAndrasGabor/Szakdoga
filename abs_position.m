device = serialport("COM8",115200);

for i=1:60

 sensordata = readline(device);
end

 while 1
 sensordata = readline(device);
 Readings = strrep(sensordata,',',' ');
 Read_All= str2num(Readings);
 Acc_X=Read_All(1)*19,8;
 Acc_Y=Read_All(2)*19,8;
 Acc_Z=Read_All(3)*19,8;
 Gyro_X=Read_All(4);
 Gyro_Y=Read_All(5);
 Gyro_Z=Read_All(6);
 Mag_X=Read_All(7);
 Mag_Y=Read_All(8);
 Mag_Z=Read_All(9);
 Time = 0.125;
 if i==60
     figure;
     i=i+1;

 end

 Z_now=[Acc_X Acc_Y Acc_Z];
 Y_now=[Mag_X Mag_Y Mag_Z];
 X_now=cross(Z_now,Y_now);
 plot3([0,Acc_X],[0,Acc_Y],[0,Acc_Z]);
 hold on;
 plot3([0,Mag_X],[0,Mag_Y],[0,Mag_Z]);
 hold on;
 plot3([0,X_now(1)],[0,X_now(2)],[0,X_now(3)]);
 xlim([-50 50]);
 ylim([-50 50]);
 zlim([-50 50]);
 hold off

 % Base_XYZ=[1 0 0; 0 1 0; 0 0 1];
 % Now_XYZ=[Acc_X Acc_Y Acc_Z;Mag_X Mag_Y Mag_Z; X_now(1) X_now(2) X_now(3)];
 % Rotmat=Base_XYZ*Now_XYZ;
 % Q_Rot_arg=rotm2quat(Rotmat);
 % Q_Rot=quaternion(Q_Rot_arg(1),Q_Rot_arg(2),Q_Rot_arg(3),Q_Rot_arg(4));
 % Q_Base=quaternion(1,0,0,0);
 % Q_abs=quatmultiply(Q_Base,Q_Rot);
 % if i==60
 %    figure;
 %    poseplot
 %    xlabel("Észak-x (m)")
 %    ylabel("Kelet-y (m)")
 %    zlabel("Lent-z (m)")
 %    i=i+1;
 % end
 % position = [1 1 1];
 % poseplot(Q_Rot,position,"enu")
 % hold off


 end

