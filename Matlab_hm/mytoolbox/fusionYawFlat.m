% Fusion filter that gives the orientation on a flat surface
% Input units are: g, °/s, µT
function yawOut= fusionYawFlat(acc,gyr,yawIn,alpha,dt)
    yawOut = alpha*(yawIn + gyr(3)*dt)+(1-alpha)*atan2(-acc(1),-acc(2))*(180/pi);
