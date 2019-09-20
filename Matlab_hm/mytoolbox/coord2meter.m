function d = coord2meter(lat1, lon1, lat2, lon2)
% haversine formula that gives the distance in meter between two GPS position
    R = 6378.137; % Radius of earth in km
    dLat = lat2 * pi / 180 - lat1 * pi / 180;
    dLon = lon2 * pi / 180 - lon1 * pi / 180;
    a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * pi / 180) * cos(lat2 * pi / 180) * sin(dLon/2) * sin(dLon/2);
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = R * c * 1000; % meters
end