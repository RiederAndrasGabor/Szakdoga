clear;
device = serialport("COM11",115200);

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
Acc_X=Acc_X/26.3;
Acc_Y=Acc_Y/26.3;
Acc_Z=Acc_Z/26.3;
Accelerometer=[Acc_X,Acc_Y,Acc_Z];
Mag_X=Mag_X-4.1;
Mag_Y=Mag_Y+28;
Mag_Z=Mag_Z-13;
Magnetometer=[Mag_X,Mag_Y,Mag_Z];
M=sqrt(Mag_X^2+Mag_Y^2+Mag_Z^2);
B=[Mag_X/M Mag_Y/M Mag_Z/M];

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
Magnetometer(2)=Mag_Y;
Magnetometer(3)=Mag_Z;


GyroCovariance=[0.0257,0.0201,0.0254];
giro_drift_cov=[0.2062,0.1011,0.0215];
GyroBias=[0.001 0.0015 -0.00046];
GyroError=[0.4,0.9,0.9];
AbsError=[0.4,0.3,0.3];

if i==61
 MADG = MadgwickAHRS('SamplePeriod', 1/14,'Quaternion',Q_abs,'Beta', 0.7);
 MAHO = Mahony('SamplePeriod', 1/14,'Quaternion',Q_abs, 'Kp', 1.5,'Ki',0.03);
 COMP = complementary('Alpha',0.1);
 KALM = KalmanFilter('SamplePeriod', 1/14,'GyroCovariance',GyroCovariance ,'GyroBias',GyroBias,'GyroError',GyroError,'AbsError',AbsError);
 i=0;
end

COMP.Update(Q_rel,Q_abs);
Q_comp= COMP.Quaternion;
KALM.Update(Gyroscope,Q_abs);
Q_kal= KALM.Quaternion;
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
pause(0.0001);
hold off

end