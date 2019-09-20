function [new_lat, new_lng] = meter2coord(lat, lng, dLat, dLon)
% haversine formula that gives the distance in meter between two GPS position
    R = 6378.137; % Radius of earth in km 
    m = (1 / ((2 * pi / 360) * R)) / 1000;  % 1 meter in degree
    
    new_lat = lat + (dLat * m);
   
    new_lng = lng + (dLon * m) / cos(lat * (pi / 180));
end