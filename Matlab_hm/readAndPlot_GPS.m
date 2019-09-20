% === Read and plot raw GPS latitude and longitude =
% Arduino : TR_nano_kalman2D.ino and RE_uno_kalman2D.ino must be peviously uploaded

clear;
close all;
clc;
addpath('mytoolbox');      

% Port reset: instrfind reads serial port objects from memory to MATLAB workspace
if not(isempty(instrfind)) 
    fclose(instrfind);
    delete(instrfind);
end


%% Serial open
arduino=serial('COM3','BaudRate',115200);
fopen(arduino);
fs = 200;  % 200Hz sample rate (refer to arduino code)
T = 1/fs;  % sample period
fc = 1.5;  % cut off frequency for LPF

y = char.empty;
while strcmp(y,char([71,79,13,10])) ~= 1
    y = char(fscanf(arduino));
    disp(y);
end


%% Read and plot GPS data
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
t0 = clock;
dt = 0;

while true 
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp) >= [4,3])
        lat = cat(1,lat,temp(4,1));
        lng = cat(1,lng,temp(4,2));
        disp([lat,lng]);
        addpoints(l,lng(end),lat(end));
        drawnow limitrate
        dt = etime(clock,t0);
    end
end

save('gps_raw_values3','lat','lng');

webmap opentopomap     % Open a web map
wmline(lat,lng,'LineWidth',8,'Color','r')     % Plot the glider path track on the basemap
