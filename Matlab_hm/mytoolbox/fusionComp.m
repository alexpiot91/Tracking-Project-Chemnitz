% Fusion filter that gives the orientation based on accelerometer and 
% gyroscope. 
% Input units are: g, °/s, µT
function [rollOut, pitchOut, yawOut]= fusionComp(acc,gyr,mag,rollIn,pitchIn,alpha,dt)
    a_x = acc(1);
    a_y = acc(2);
    a_z = acc(3);
    g_x = gyr(1);
    g_y = gyr(2);

% I. Roll (around X) and Pitch (around Y)
    accRoll = atan2(a_y,a_z^2); 
    % atan2 because -pi<roll<pi
    accPitch = atan(-a_x/sqrt(a_y^2+a_z^2)); 
    % atan because -pi/2<roll<pi/2

%     % I. Roll (around X) and Pitch (around Y)
%     accRoll = atan2(acc(2),sign(acc(3))*sqrt(mu*acc(1)^2+acc(3)^2)); 
%     % atan2 because -pi<roll<pi
%     accPitch = atan2(-acc(1),sqrt(acc(2)^2+acc(3)^2)); 
%     % atan because -pi/2<roll<pi/2
    
    
    rollOut = alpha*(g_x*dt+rollIn)+(1-alpha)*accRoll*(180/pi);
    pitchOut = alpha*(g_y*dt+pitchIn)+(1-alpha)*accPitch*(180/pi);
    
% II. Yaw (around Z)
    %magN = norm(mag);
    m_x = mag(2);%/magN;
    m_y = mag(1);%/magN;
    m_z = -mag(3);%/magN;
    
    Yh = -m_y*cos(accRoll)+m_z*sin(accRoll);
    Xh = m_x*cos(accPitch)+m_y*sin(accPitch)*sin(accRoll)+m_z*sin(accPitch)*cos(accRoll);
    yawOut = atan2(Yh,Xh)*(180/pi);
    % atan2 because -pi<yaw<pi

