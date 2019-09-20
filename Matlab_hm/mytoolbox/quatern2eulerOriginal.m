function euler = quatern2eulerOrigin(q)
% For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw  = atan2(2*q(2)*q(3) - 2*q(1)*q(4), 2*q(1)*q(1) + 2*q(2)*q(2) - 1);   
    pitch = -asin(2*q(2)*q(4) + 2*q(1)*q(3));
    roll = atan2(2*q(3)*q(4) - 2*q(1)*q(2), 2*q(1)*q(1) + 2*q(4)*q(4) - 1);
    
    euler = [roll, pitch, yaw];