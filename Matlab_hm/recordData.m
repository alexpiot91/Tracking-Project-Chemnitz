% === Record data for further use ===
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
magList = zeros([0 3]);
gps1List = zeros([0 3]); % lat, lng, 0
gps2List = zeros([0 3]); % head, vel, alt
time = zeros([0 1]);

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
l = animatedline('Color','r','Marker','.');
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

% Read and record
t = 0;
update = clock;
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [5,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gyr = temp(2,:) - gyrOff;
        mag = temp(3,:);
        gps1 = temp(4,:);
        disp(gps1);
        gps2 = temp(5,:);
        deltat = etime(clock,update);    % calclate deltat
        update = clock;
        %disp(gps2);

        accList = cat(1, accList, acc);
        gyrList = cat(1, gyrList, gyr);
        magList = cat(1, magList, mag);
        gps1List = cat(1, gps1List, gps1);
        gps2List = cat(1, gps2List, gps2);
        time = cat(1, time, deltat);
        
        addpoints(l,gps1(2),gps1(1)); % lng,lat
        drawnow limitrate
    end
end

save('record_data3','time','accList','gyrList','magList','gps1List','gps2List');

webmap opentopomap     % Open a web map
wmline(gps1List(:,1),gps1List(:,2),'LineWidth',3,'Color','r'); % lat,lng  