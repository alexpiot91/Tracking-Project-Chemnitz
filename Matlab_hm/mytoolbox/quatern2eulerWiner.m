function euler = quatern2eulerWiner(q)
% For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2 * (q(2) * q(3) + q(1) * q(4)), q(1) * q(1) + q(2) * q(2) - q(3) * q(3) - q(4) * q(4));   
    pitch = -asin(2 * (q(2) * q(4) - q(1) * q(3)));
    roll  = atan2(2 * (q(1) * q(2) + q(3) * q(4)), q(1) * q(1) - q(2) * q(2) - q(3) * q(3) + q(4) * q(4));
    
    euler = [yaw, pitch, roll];
    