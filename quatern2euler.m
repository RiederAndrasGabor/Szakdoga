function euler = quatern2euler(q)
    [q0,q1,q2,q3]=parts(q);
    roll=atan2(2*(q0*q1+q2*q3),(q0^2-q1^2-q2^2+q3^2));
    pitch=asin(2*(q0*q2-q3*q1));
    yaw=atan2(2*(q0*q3+q1*q2),(q0^2+q1^2-q2^2-q3^2));
    
    if pitch==90
        yaw=-2*atan2(q1,q0);
        roll=0;
    end
    if pitch==-90
        yaw=2*atan2(q1,q0);
        roll=0;
    end
    euler = [yaw pitch roll];
end

