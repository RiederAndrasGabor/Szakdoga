classdef complementary < handle
    %% Publikus változó
    properties (Access = public)
        alpha = 0.1;  %1- abszolút orientáció; 0-relatív orientáció
        Quaternion = quaternion(1,0,0,0);     % frame-ek közti forgatás
    end
    %% Publikus függvények
    methods (Access = public)
        function obj = complementary(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'Alpha'), obj.alpha = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Q_rel, Q_abs)
        obj.Quaternion=obj.alpha*Q_abs+(1-obj.alpha)*Q_rel;
        end
    end
end


%QUATERNIÓKKAL


% Eul_Angles1=quat2eul(Q_Rot); 
% yaw_abs= Eul_Angles1(1);
% pitch_abs=Eul_Angles1(2);
% roll_abs=Eul_Angles1(3);
% Eul_Angles2=quat2eul(Q_rel); 
% yaw_rel= Eul_Angles2(1);
% pitch_rel=Eul_Angles2(2);
% roll_rel=Eul_Angles2(3);