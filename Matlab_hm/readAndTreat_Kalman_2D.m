% === Read and plot raw GPS latitude and longitude =
% Arduino : TR_nano_kalman2D.ino and RE_uno_kalman2D.ino must be peviously uploaded

% "doc" to see a function's way of working
% "edit" to see how is written a function

clear;
close all;
clc;
addpath('mytoolbox');     

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
gyrList = zeros([0 3]);
x_list = zeros([0 4]);
lat_list = zeros([0 1]);
lng_list = zeros([0 1]);
head_list = zeros([0 1]);
vel_list = zeros([0 1]);

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
l = animatedline('Color','r','Marker','.');
title('Crossing points of the 2D Kalman Filter');
xlabel('longitude (°)');
ylabel('latitude (°)');
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

% Data acquisition for calibration
t = 0;
t0 = clock;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        acc = temp(1,:);
        gyr = temp(2,:);
        
        % Build lists before offset calculation
        accList = cat(1,accList,acc);
        gyrList = cat(1,gyrList,gyr);

        t = clock - t0;
    end
end

% Calibration calculation
accOff = mean(accList);
gyrOff = mean(gyrList);

% Kalman filter initialization
std_lat = 1.7;
std_lng = 1.3;
std_vel = 0.05;
std_head = 0.3*pi/180;
std_yrate = 0.1*pi/180;
std_alng = 0.01;
std_alat = 0.01;

H = [1, 0, 0, 0, 0, 0;  % Linearized measurement model matrix H
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1;
    0, 0, 0, 1, 1, 0];

R = [std_lat^2, 0, 0, 0, 0, 0, 0;  % Measurement covariance matrix R
    0, std_lng^2, 0, 0, 0, 0, 0;
    0, 0, std_head^2, 0, 0, 0, 0;
    0, 0, 0, std_vel^2, 0, 0, 0;
    0, 0, 0, 0, std_yrate^2, 0, 0;
    0, 0, 0, 0, 0, std_alng^2, 0;
    0, 0, 0, 0, 0, 0, std_alat^2];


Q = [10*(20*pi/180)^2, 0;  % Process noise covariance matrix Q
    0, 1000];

x = zeros([6 1]);  % Initialization of x and P              
P = R(1:6,1:6);

% Starting point 
lat_old = zeros([0 0]);
lng_old = zeros([0 0]);
while isempty(lat_old)
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    size_temp = size(temp);
    if (size_temp >= [4,3])  
        gps = temp(4,:);
        coord_init = [gps(1),gps(2)]; % starting point of our trip
        lat_old = gps(1); % used as a reference for distance calculations
        lng_old = gps(2); % used as a reference for distance calculations
    end
end

% Read and treat
t = 0;
update = clock;
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [5,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gyr = temp(2,:) - gyrOff;
        gps = temp(4,:);
        gps2 = temp(5,:);
        %disp(gps2);

        % Process sensor data through algorithm
        deltat = etime(clock,update);    % calclate deltat
        update = clock;                  % initialize next deltat
        z = [coord2meter(0, gps(2), 0, lng_old);  % longitude (East shifting) measured in m
            coord2meter(gps(1), 0, lat_old, 0);   % latitude (North shifting) measured in m
            gps2(1)*pi/180;                       % heading in rad
            gps2(2);                              % velocity in m/s
            gyr(3)*pi/180;                        % yaw rate in rad/s
            acc(1);                               % longitudinal acceleration in m/s^2
            acc(2)];                              % lateral acceleration in m/s^2  
        %disp(z);
        [x_upd, P_upd] = fusionKalman2D(x, P, z, H, R, Q, deltat);
        x = x_upd;
        P = P_upd;
        lat_old = gps(1);
        lng_old = gps(2);
        
        % display
        [new_lat, new_lng] = meter2coord(coord_init(1), coord_init(2), x(2), x(1));
        x_list = cat(1, x_list, [new_lng, new_lat, x(3), x(4)]);  % lng,lat,head,vel
        addpoints(l,x_list(end, 1),x_list(end, 2));
        hold on
        %plot([x_list(end, 1),x_list(end, 1)+0.00001*sin(x_list(end,3))],[x_list(end, 2),x_list(end, 2)+0.00001*cos(x_list(end,3))],'-b');
        drawnow limitrate
    end
end

save('kalman2D_values','x_list');

webmap opentopomap     % Open a web map
wmline(x_list(:,2),x_list(:,1),'LineWidth',3,'Color','r')     % Plot the glider path track on the basemap