device = serialport("COM8",115200);
for i=1:60

 sensordata = readline(device);
end
i=0;
Mag_Datas=zeros(1500,3);
 for i=1:1500
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
 Mag_X=Mag_X-4,1;
 Mag_Y=Mag_Y+28;
 Mag_Z=Mag_Z-13;
 Mag_Datas(i,1)=Mag_X;
 Mag_Datas(i,2)=Mag_Y;
 Mag_Datas(i,3)=Mag_Z;

 if i==0
    figure;
    xlabel("Észak-x (m)")
    ylabel("Kelet-y (m)")
    zlabel("Lent-z (m)")
    i=i+1;
    axis equal
title('Magnetometer Data With Hard and Soft Iron Effects')
 end
scatter3(Mag_X,Mag_Y,Mag_Z);
hold on;

i=i+1;
 end

[A,b,expMFS]  = magcal(Mag_Datas);
xCorrected = (Mag_Datas-b)*A;