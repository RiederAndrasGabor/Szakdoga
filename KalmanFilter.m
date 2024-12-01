classdef KalmanFilter < handle
    properties (Access = public)
        SamplePeriod = 1/256;
        GyroCovariance = [0,0,0];
        GyroBias=[0,0,0];
        GyroError=[0,0,0];
        AbsError=[0,0,0];
        Quaternion = quaternion(1,0,0,0);     % frame-ek közti forgatás
    end
    properties (Access = private)
        yaw_measured=0;
        roll_measured=0;
        pitch_measured=0;
        yaw_covariance_measured=0;
        roll_covariance_measured=0;
        pitch_covariance_measured=0;
        firstRun=1;
    end

    %% Függvények
    methods (Access = public)
        function obj = KalmanFilter(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'GyroCovariance'), obj.GyroCovariance = varargin{i+1};
                elseif  strcmp(varargin{i}, 'GyroBias'), obj.GyroBias = varargin{i+1};
                elseif  strcmp(varargin{i}, 'GyroError'), obj.GyroError = varargin{i+1};
                elseif  strcmp(varargin{i}, 'AbsError'), obj.AbsError = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end
        function obj = Update(obj, Gyroscope, Q_abs)

        %Euler szögek meghatározása
        Eul_Angles1=quatern2euler(Q_abs);%,"ZYX"); 
        yaw_abs= Eul_Angles1(1);
        pitch_abs=Eul_Angles1(2);
        roll_abs=Eul_Angles1(3);
    
        if (1==obj.firstRun)
            yaw_current=yaw_abs;
            roll_current=roll_abs;
            pitch_current=pitch_abs;
            yaw_covariance_current=obj.GyroCovariance(1);
            roll_covariance_current=obj.GyroCovariance(2);
            pitch_covariance_current=obj.GyroCovariance(3);
            obj.firstRun = 0;  
        else
            yaw_current=obj.yaw_measured;
            roll_current=obj.roll_measured;
            pitch_current=obj.pitch_measured;
            yaw_covariance_current=obj.yaw_covariance_measured;
            roll_covariance_current=obj.roll_covariance_measured; 
            pitch_covariance_current=obj.pitch_covariance_measured; 
        end

    %mérés predikció
        yaw_predicted=yaw_current+obj.SamplePeriod*(Gyroscope(1)-obj.GyroBias(1));
        roll_predicted=roll_current+obj.SamplePeriod*(Gyroscope(2)-obj.GyroBias(2));
        pitch_predicted=pitch_current+obj.SamplePeriod*(Gyroscope(3)-obj.GyroBias(3));


    %kovariancia predikció, 1^2=giroszkóp eltérése
        yaw_covariance_predicted=yaw_covariance_current+(obj.SamplePeriod^2*obj.GyroError(1)^2);
        roll_covariance_predicted=roll_covariance_current+(obj.SamplePeriod^2*obj.GyroError(2)^2);
        pitch_covariance_predicted=pitch_covariance_current+(obj.SamplePeriod^2*obj.GyroError(3)^2);
        % yaw_covariance_predicted=yaw_covariance_current+(obj.SamplePeriod^2*0.9^2);
        % roll_covariance_predicted=roll_covariance_current+(obj.SamplePeriod^2*0.7^2);
        % pitch_covariance_predicted=pitch_covariance_current+(obj.SamplePeriod^2*0.7^2);

    %Kálmán erősítés számítása; kb 0.3 FOK HIBÁJA LEHET KB AZ ABSZ POZíCIÓ MÉRÉSNEK
        % kalman_gain_yaw=yaw_covariance_predicted/(yaw_covariance_predicted+0.4^2);
        % kalman_gain_roll=roll_covariance_predicted/(roll_covariance_predicted+0.3^2);
        % kalman_gain_pitch=pitch_covariance_predicted/(pitch_covariance_predicted+0.3^2);
        kalman_gain_yaw=yaw_covariance_predicted/(yaw_covariance_predicted+obj.AbsError(1)^2);
        kalman_gain_roll=roll_covariance_predicted/(roll_covariance_predicted+obj.AbsError(1)^2);
        kalman_gain_pitch=pitch_covariance_predicted/(pitch_covariance_predicted+obj.AbsError(1)^2);
    % A számított orientáció
        obj.yaw_measured=yaw_predicted+kalman_gain_yaw*(yaw_abs-yaw_predicted);
        obj.roll_measured=roll_predicted+kalman_gain_roll*(roll_abs-roll_predicted);
        obj.pitch_measured=pitch_predicted+kalman_gain_pitch*(pitch_abs-pitch_predicted);


    % A számított kovariancia
        obj.yaw_covariance_measured= (1-kalman_gain_yaw)*yaw_covariance_predicted;
        obj.pitch_covariance_measured= (1-kalman_gain_pitch)*pitch_covariance_predicted;
        obj.roll_covariance_measured= (1-kalman_gain_roll)*roll_covariance_predicted;


        EulerAngles=[obj.yaw_measured obj.pitch_measured  obj.roll_measured];
        Q_rot= eul2quat(EulerAngles);
        obj.Quaternion= quaternion(Q_rot(1),Q_rot(2),Q_rot(3),Q_rot(4));
        end
    end
end
