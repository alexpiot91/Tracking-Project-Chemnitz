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


%% Standard deviation calculation for the IMU
% == Make sure the IMU is at rest in an area without magnetic disturbance ==
% Initialize data
accList = zeros([0 3]);
gyrList = zeros([0 3]);
magList = zeros([0 3]);

% Wait for stabilization
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    t = t + T;
end

% Initialization of Kalman filter
while length(accList) < 400
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])  
        acc = temp(1,:);
        gyr = temp(2,:);
        mag = temp(3,:);
        accList = cat(1,accList,acc);
        gyrList = cat(1,gyrList,gyr);
        magList = cat(1,magList,mag);
    end
end

accstd = std(accList);
accXstd = accstd(1); accYstd = accstd(2); accZstd = accstd(3);
gyrstd = std(gyrList);
gyrXstd = gyrstd(1); gyrYstd = gyrstd(2); gyrZstd = gyrstd(3);
magstd = std(magList);
magXstd = magstd(1); magYstd = magstd(2); magZstd = magstd(3);

save('std_dev_imu','accXstd','accYstd','accZstd','gyrXstd','gyrYstd','gyrZstd','magXstd','magYstd','magZstd');


%% Standard deviation calculation for the GPS
% == Make sure the GPS is in an area where GPS signal is available ==
% Initialize data
latList = zeros([0 1]);
lngList = zeros([0 1]);
velList = zeros([0 1]);

% Wait for stabilization
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    t = t + T;
end

% Initialization of Kalman filter
while length(latList) < 50
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp == [4,3])  
        gps = temp(4,:);
        latList = cat(1,latList,gps(1));
        lngList = cat(1,lngList,gps(2));
        velList = cat(1,velList,gps(3));
        disp(gps);
    end
end

latstd = std(latList); lngstd = std(lngList); velstd = std(velList);

save('std_dev_gps','latstd','lngstd','velstd');