% motionModel = np.array([[1,-1*dt],[0,1]])
% 
% prediction = np.matmul(motionModel,currentState) + dt*(np.vstack((angularVelocity,0.0)))
% 
% errorMatrix = np.array([error, driftError])*np.identity(2)
% predictedCovariance = np.matmul(np.matmul(motionModel, currentCovariance), (motionModel.T)) + errorMatrix
% ,
% 
% difference = measurement - np.matmul(np.array([1.0, 1.0]), prediction)
% 
% 		measurementCovariance = np.matmul(np.matmul(np.array([1.0, 0.0]), predictedCovariance),np.vstack((1.0,0.0))) + measurementError
% 		kalmanGain = np.matmul(predictedCovariance, np.vstack((1.0, 0.0)))/measurementCovariance
% 
% 		correctedState = prediction + kalmanGain*(measurement - np.matmul(np.array([1.0, 0.0]), prediction))
% 		% correctedState = prediction + kalmanGain*(difference)
% 
% 		updatedCovariance = np.matmul( np.identity(2) - np.matmul(kalmanGain, np.array([1.0, 0.0]).reshape((1,2))), predictedCovariance)


% classdef kalman_szuro
%     properties
%     time {mustBeNumeric}
%     end 
%      methods
%         function Q-rot=getrot()
%             Transf_matrix=[1 -time;0 1];
%             Prediction=Transf_matrix*Gyro_X+
% 
%     end
%      end



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

time=0.075;
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
% yaw_covariance_current=[giro_cov(1) 0;0 giro_drift_cov(1)];
% roll_covariance_current=[giro_cov(2) 0;0 giro_drift_cov(2)];
% pitch_covariance_current=[giro_cov(3) 0;0 giro_drift_cov(3)];

if i~=61
 yaw_now=yaw_measured;
 roll_now=roll_measured;
 pitch_now=pitch_measured;
 yaw_covariance_current=yaw_covariance_measured;
 roll_covariance_current=roll_covariance_measured; 
 pitch_covariance_current=pitch_covariance_measured; 
 %Q_comp=Q_abs;
end

yaw_pred=yaw_now+time*(yaw_rel-bias_t_prev(1));
roll_pred=roll_now+time*(roll_rel-bias_t_prev(2));
pitch_pred=pitch_now+time*(pitch_rel-bias_t_prev(3));


%covariance_current=[yaw_covariance_current roll_covariance_current pitch_covariance_current];
%1^2=giroszkóp eltérése
yaw_covariance_predicted=yaw_covariance_current+(time^2*1^2);
roll_covariance_predicted=roll_covariance_current+(time^2*1^2);
pitch_covariance_predicted=pitch_covariance_current+(time^2*1^2);
%covariance_predicted=[yaw_covariance_predicted,roll_covariance_predicted,pitch_covariance_predicted];


% Transf_matrix=[1 -time;0 1];
% 
% %prediction 
% yaw_pred=yaw_now+time*(yaw_rel-bias_t_prev(1));
% roll_pred=roll_now+time*(roll_rel-bias_t_prev(2));
% pitch_pred=pitch_now+time*(pitch_rel-bias_t_prev(3));
% 
% error_matrix=[giro_bias 0;0 giro_drift_bias];
% 
% covariance_current=[yaw_covariance_current roll_covariance_current pitch_covariance_current];
% yaw_covariance_predicted=Transf_matrix*yaw_covariance_current*(Transf_matrix.')+error_matrix;
% roll_covariance_predicted=Transf_matrix*roll_covariance_current*(Transf_matrix.')+error_matrix;
% pitch_covariance_predicted=Transf_matrix*pitch_covariance_current*(Transf_matrix.')+error_matrix;
% covariance_predicted=[yaw_covariance_predicted,roll_covariance_predicted,pitch_covariance_predicted];

% % Set Reference Magnetic vector (NORMALIZATION)
% M=sqrt((Mag_X)^2+(Mag_Y)^2+(Mag_Z)^2);
% B=[Mag_X/M Mag_Y/M Mag_Z/M ];
% %mérés...
% %MAIN EKF FUNCTION
% [q0, q1, q2, q3] = Kalman(pitch_now, roll_now, yaw_now, B, Mag_X, Mag_Y, Mag_Z, Acc_X, Acc_Y, Acc_Z, time,1, 1,1);
% %Q_comprot_1= quaternion(q0, q1, q2, q3);
% % Q_comp=quatmultiply(Q_comp,Q_comprot,'ReferenceFrame','enu');
% % Q_comprot=quatinv(Q_comprot);
% % position = [7 7 7];
% % poseplot(Q_comprot,position,"enu")
% % hold off

%1 FOK HIBÁJA LEHET KB AZ ABSZ POZíCIÓ MÉRÉSNEK
kalman_gain_yaw=yaw_covariance_predicted/(yaw_covariance_predicted+0.3^2);
kalman_gain_roll=roll_covariance_predicted/(roll_covariance_predicted+0.3^2);
kalman_gain_pitch=pitch_covariance_predicted/(pitch_covariance_predicted+0.3^2);

% kalman predikcio
yaw_measured=yaw_pred+kalman_gain_yaw*(yaw_abs-yaw_pred);
roll_measured=roll_pred+kalman_gain_roll*(roll_abs-roll_pred);
pitch_measured=pitch_pred+kalman_gain_pitch*(pitch_abs-pitch_pred);


% kovariancia update
yaw_covariance_measured= (1-kalman_gain_yaw)*yaw_covariance_predicted;
pitch_covariance_measured= (1-kalman_gain_pitch)*pitch_covariance_predicted;
roll_covariance_measured= (1-kalman_gain_roll)*roll_covariance_predicted;

% Q_Kalman= quaternion([yaw_measured pitch_measured  roll_measured],"eulerd","XYZ","frame");
% position = [7 7 7];
% poseplot(Q_Kalman,position,"enu")
% hold off
% i=i+1;

eul=[-yaw_measured -pitch_measured  -roll_measured];
Q_rot= eul2quat(eul);
Q_comprot= quaternion(Q_rot(1),Q_rot(2),Q_rot(3),Q_rot(4));
Q_comp=quatmultiply(Q_comp,Q_comprot,'ReferenceFrame','enu');
Q_comprot=quatinv(Q_comprot);
position = [7 7 7];
poseplot(Q_comprot,position,"enu")
%pause(0.0001);
drawnow;
hold off

% yaw_diff= yaw_abs-yaw_pred;
% roll_diff= roll_abs-roll_pred;
% pitch_diff= pitch_abs-pitch_pred;
% 
% %kalman_gain1=yaw_covariance_predicted*(inv(yaw_covariance_predicted*yaw_covariance_measured));
% H=[1 0];
% yaw_covariance_measured=(H*yaw_covariance_predicted*(H.'))+meresi_hibak(1);
% roll_covariance_measured=(H*roll_covariance_predicted*(H.'))+meresi_hibak(2);
% pitch_covariance_measured=(H*pitch_covariance_predicted*(H.'))+meresi_hibak(3);
% covariance_measured=[yaw_covariance_measured, roll_covariance_measured,pitch_covariance_measured];
% %kalman_gain=covariance_predicted*(H.')*inv(covariance_measured);
% kalman_gain1=yaw_covariance_predicted/yaw_covariance_measured;
% kalman_gain2=roll_covariance_predicted/roll_covariance_measured;
% kalman_gain3=pitch_covariance_predicted/pitch_covariance_measured;
% 
% yaw_corrected=yaw_pred+(kalman_gain1(2,2)*yaw_diff);
% roll_corrected=roll_pred+(kalman_gain2(2,2)*roll_diff);
% pitch_corrected=pitch_pred+(kalman_gain3(2,2)*pitch_diff);
% 
% covariance_updated=covariance_predicted-(kalman_gain*H*covariance_predicted);
% 
% yaw_now=yaw_corrected;
% roll_now=roll_corrected;
% pitch_now=pitch_corrected;


 end
