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
arduino=serial('COM3','BaudRate',250000);
fopen(arduino);
fs = 200;  % 200Hz sample rate (refer to arduino code)
T = 1/fs;  % sample period

y = char.empty;
while strcmp(y,char([71,79,13,10])) ~= 1  
    % while y~='GO', stay in the loop
    y = char(fscanf(arduino));
    disp(y);
end


%% Read data and 3D
% Initialize data
load('mag_calib_values');
gyrList = zeros([0 3]);
accList = zeros([0 3]);
mag_dec = -3.92*(pi/180); % use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml with [50.8160, 12.9293]
lat_list = zeros([0 1]);
lng_list = zeros([0 1]);
yaw_list = zeros([0 1]);

% Initialize filter
beta = 0.6046;
zeta = 0.005;
result = [1 0 0 0 0 0 0];

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

% Initialize data
lat = zeros([0 1]);
lng = zeros([0 1]);

% Wait for stabilization
t = 0;
t0 = clock;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    t = clock - t0;
end

% Data acquisition for calibration
t = 0;
t0 = clock;
while t<1.5
    y = fgets(arduino);
    temp = str2num(y);
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

% Read and treat
t = 0;
update = clock;
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gyr = temp(2,:) - gyrOff;
        mag = temp(3,:)-[x_avg,y_avg,z_avg];

        % Process sensor data through algorithm
        deltat = etime(clock,update);                                   % calclate deltat
        update = clock;                                                 % initialize next deltat
        result = fusionMadgwick(result, acc, gyr*(pi/180), mag, beta, zeta, deltat);	% gyroscope units must be radians
        q0 = result(1:4);  % initialize next quaternion
        euler = quatern2eulerWiner(q0);                            % calculate euler angles [yaw, pitch, roll]
        disp(euler.*(180/pi));
        
        if (size(temp) >= [4,3])
            lat = temp(4,1);
            lng = temp(4,2);
            lat_list = cat(1,lat_list,temp(4,1));
            lng_list = cat(1,lng_list,temp(4,2));
            yaw_list = cat(1,yaw_list,euler(1));
            disp([lat,lng]);
            addpoints(l,lng(end),lat(end));
            hold on
            plot([lng,lng+0.0001*cos(euler(1)+mag_dec)],[lat,lat+0.0001*sin(euler(1)+mag_dec)],'-b');
            %xlswrite('data',[lat,lng,euler(1)],'append');
            drawnow limitrate
        end
    end
end

save('madgwick_gps_values3','lat_list','lng_list','yaw_list');

webmap opentopomap     % Open a web map
wmline(lat_list,lng_list,'LineWidth',3,'Color','r')     % Plot the glider path track on the basemap