classdef Mahony < handle
    %% Publikus változó
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = quaternion(1,0,0,0);     % frame-ek közti forgatás
        Kp = 1;                     % Proporcionális tag
        Ki = 0;                     % Integrális tag
    end
    
    %% Privát változó
    properties (Access = private)
        eInt = [0 0 0];             % Integrálási hiba
    end    
 
    %% Publikus függvények
    methods (Access = public)
        function obj = Mahony(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            Q_former = obj.Quaternion; 
            if(norm(Accelerometer) == 0), return; end	% NaN esetére
            Accelerometer = Accelerometer / norm(Accelerometer);	

            if(norm(Magnetometer) == 0), return; end	% NaN esetére
            Magnetometer = Magnetometer / norm(Magnetometer);	

            % Magnetométer értékek tool keretbe helyezése
            Q_mag=quaternion(0,Magnetometer(1),Magnetometer(2),Magnetometer(3));
            h = quatmultiply(Q_former,Q_mag);
            [h0,h1,h2,h3] = parts(h);
            b2=norm([h1 h2]);
            b = [0 b2 0 h3];
            [q(1),q(2),q(3),q(4)] = parts(Q_former);
            
            % Gravitációs és mágneses tér irányai
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
            w = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3))
                 2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4))
                 2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2)]; 
 
            % A hiba értéke a prediktált és a mért értékek keresztszorzata
            e = cross(Accelerometer, v) + cross(Magnetometer, w); 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;          
            Q_Gyro=quaternion(0, Gyroscope(1), Gyroscope(2), Gyroscope(3));
            qDot = 0.5 * quatmultiply(Q_former, Q_Gyro);
            Q_former = Q_former + qDot * obj.SamplePeriod;
            obj.Quaternion = Q_former / norm(Q_former);
        end
    end
end