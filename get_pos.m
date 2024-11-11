classdef get_pos < handle
    properties (Access = public)
        SamplePeriod = 1/14;
    % end
    % %% Private properties
    % properties (Access = private)
        Q_Pos_Abs= quaternion(1,0,0,0);
        Q_Pos_Rel= quaternion(1,0,0,0);
    end 
    %% Függvények
    methods (Access = public)
        function obj = get_pos(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end
        function obj = Get_Pos_Abs(obj,Accelerometer, Magnetometer)
            Z_now=[Accelerometer(1) Accelerometer(2) Accelerometer(3)];
            Z_now = Z_now/norm(Z_now);
            Y_now=[Magnetometer(1) Magnetometer(2) Magnetometer(3)];
            Y_now = Y_now/norm(Y_now);
            X_now=cross(Y_now,Z_now);
            Y_now2=cross(Z_now,X_now);
            Current_XYZ=[X_now', Y_now2', Z_now'];
            Rotmat=Current_XYZ^-1;
            Q_Rot_arg=rotm2quat(Rotmat);
            Q_Rot=quaternion(Q_Rot_arg(1),Q_Rot_arg(2),Q_Rot_arg(3),Q_Rot_arg(4));
            Q_Base=quaternion(1,0,0,0);
            obj.Q_Pos_Abs=quatmultiply(Q_Base,Q_Rot,'ReferenceFrame','enu');
        end
        function obj = Get_Pos_Rel(obj,Q_former,Gyroscope)
            Gyroscope(1)=Gyroscope(1)*obj.SamplePeriod;
            Gyroscope(2)=Gyroscope(2)*obj.SamplePeriod;
            Gyroscope(3)=Gyroscope(3)*obj.SamplePeriod;
            Q_Gyro= quaternion([Gyroscope(1) Gyroscope(2) Gyroscope(3)],"eulerd","XYZ","frame");
            obj.Q_Pos_Rel= quatmultiply(Q_former,Q_Gyro,'ReferenceFrame','enu');
        end
    end
end