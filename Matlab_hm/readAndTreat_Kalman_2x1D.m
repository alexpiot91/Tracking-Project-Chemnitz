clear;
close all;
clc;
addpath('mytoolbox');     

% "doc" to see a function's way of working
% "edit" to see how is written a function

% Port reset:
% instrfind reads serial port objects from memory to MATLAB workspace
if not(isempty(instrfind)) 
    fclose(instrfind);
    delete(instrfind);
end


%% Serial open
arduino=serial('COM3','BaudRate',115200);
fopen(arduino);
fs = 200;  % 200Hz sample rate (refer to arduino code)
T = 1/fs;  % sample period

y = char.empty;
while strcmp(y,char([71,79,13,10])) ~= 1  
    % while y~='GO', stay in the loop
    y = char(fscanf(arduino));
    disp(y);
end


%% Read data and 2D
% Initialize data
accList = zeros([0 3]);
lat_list = zeros([0 1]);
lng_list = zeros([0 1]);

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
l = animatedline('Color','r','Marker','.');
grid on
pbaspect([1 1 1]);

% Initialize map
name = 'opentopomap';     % Define the name that you will use to specify your custom basemap.
url = 'a.tile.opentopomap.org';     % Specify the website that provides the map data
copyright = char(uint8(169));     % Create an attribution to display on the map that gives credit to the provider of the map data
attribution = [ ...
      "map data:  " + copyright + "OpenStreetMap contributors,SRTM", ...
      "map style: " + copyright + "OpenTopoMap (CC-BY-SA)"];
displayName = 'Open Topo Map';     % Define the name that will appear
addCustomBasemap(name,url,'Attribution',attribution,'DisplayName',displayName)     % Add the custom basemap to the list of basemap layers available.

% Wait for stabilization
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    t = t + T;
end

% Calibration calculation
t = 0;
t0 = clock;
while t < 1
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    size_temp = size(temp);
    if (size_temp == [3,3])
        acc = temp(1,:);
        accList = cat(1,accList,acc);
    end
    t = etime(clock,t0);
end
accOff = mean(accList);

% Initialization of Kalman filter 
load('std_dev_imu');
s_accX = accXstd; % acceleration noise (m/sec^2)
s_accY = accYstd; % acceleration noise (m/sec^2)
load('std_dev_gps');
s_lat = coord2meter(0,0,latstd,0); % latitude noise (m)
s_lng = coord2meter(0,0,0,lngstd); % longitude noise (m)

x_lat = [0; 0]; % initial state vector
Q_lat = s_accX^2 * [T^4/4 T^3/2; T^3/2 T^2]; % process noise covariance matrix
P_lat = Q_lat; % initial estimation covariance matrix
lat_old = zeros([0 0]);

x_lng = [0; 0]; % initial state vector
Q_lng = s_accX^2 * [T^4/4 T^3/2; T^3/2 T^2]; % process noise covariance matrix
P_lng = Q_lng; % initial estimation covariance matrix
lng_old = zeros([0 0]);

while isempty(lat_old)
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    size_temp = size(temp);
    if (size_temp == [4,3])  
        gps = temp(4,:);
        coord_init = [gps(1),gps(2)]; % starting point of our trip
        lat_old = gps(1); % used as a reference for distance calculations
        lng_old = gps(2); % used as a reference for distance calculations
    end
end

% Read and treat
t = 0;
update = clock;
while length(lat_list) < 10
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp == [4,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gps = temp(4,:);
        disp(gps);

        % Process sensor data through algorithm
        deltat = etime(clock,update);    % calclate deltat
        update = clock;                  % initialize next deltat
        z_lat = coord2meter(gps(1), 0, lat_old, 0); % position measured in m
        [x_lat_upd, P_lat_upd] = fusionKalman1D(x_lat, P_lat, acc(1)*9.81, z_lat, s_accX, s_lat, deltat); % acc in m/s²
        x_lat = x_lat_upd;
        P_lat = P_lat_upd;
        lat_old = gps(1);
        z_lng = coord2meter(gps(1), 0, lng_old, 0); % position measured in m
        [x_lng_upd, P_lng_upd] = fusionKalman1D(x_lng, P_lng, acc(1)*9.81, z_lng, s_accY, s_lng, deltat); % acc in m/s²
        x_lng = x_lng_upd;
        P_lng = P_lng_upd;
        lng_old = gps(1);
        
        % display
        [new_lat, new_lng] = meter2coord(coord_init(1), coord_init(2), x_lat(1), x_lng(1));
        lat_list = cat(1, lat_list, new_lat);
        lng_list = cat(1, lng_list, new_lng);
        addpoints(l,lng_list(end),lat_list(end));
        drawnow limitrate
    end
end

webmap opentopomap     % Open a web map
wmline(lat_list,lng_list,'LineWidth',8,'Color','r')     % Plot the glider path track on the basemap