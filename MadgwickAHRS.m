classdef MadgwickAHRS < handle
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = quaternion(1,0,0,0);     % frame-ek közti forgatás
        Beta = 1;               	% alap erősítés
    end

    %% Függvények
    methods (Access = public)
        function obj = MadgwickAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            Q_base = obj.Quaternion; 
            if(norm(Accelerometer) == 0), return; end	% NaN esetére
            Accelerometer = Accelerometer / norm(Accelerometer);	

            if(norm(Magnetometer) == 0), return; end	% NaN esetére
            Magnetometer = Magnetometer / norm(Magnetometer);	

            % Magnetométer értékek tool keretbe helyezése
            Q_mag=quaternion(0,Magnetometer(1),Magnetometer(2),Magnetometer(3));
            h = quatmultiply(Q_base,Q_mag);
            [h0,h1,h2,h3] = parts(h);
            b2=norm([h1 h2]);
            b = [0 b2 0 h3];
            [q(1),q(2),q(3),q(4)] = parts(Q_base);
            % Gradienses algoritmus
            F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
                2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
                2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
                2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
            J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
                2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
                0,                         -4*q(2),                    -4*q(3),                         0
                -2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
            step = (J'*F);
            step = step / norm(step);	

   
            Q_Gyro=quaternion(0, Gyroscope(1), Gyroscope(2), Gyroscope(3));
            qDot = 0.5 * quatmultiply(Q_base, Q_Gyro);
            [qDot1,qDot2,qDot3,qDot4] = parts(qDot);
            qDot1=qDot1-obj.Beta*step(1);
            qDot2=qDot2-obj.Beta*step(2);
            qDot3=qDot3-obj.Beta*step(3);
            qDot4=qDot4-obj.Beta*step(4);
            qDot=quaternion(qDot1,qDot2,qDot3,qDot4);
            Q_base = Q_base + qDot * obj.SamplePeriod;
            obj.Quaternion = Q_base / norm(Q_base);
           

        end
    end
end