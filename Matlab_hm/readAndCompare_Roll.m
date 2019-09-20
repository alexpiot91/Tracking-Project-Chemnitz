clear;
close all;
clc;
addpath('mytoolbox');      

% "doc" to see a function's way of working
% "edit" to see how is written a function


%% Port reset:
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
    y = char(fscanf(arduino));
    disp(y);
end


%% Read data
% Initialize data
x_avg = -7.8246;
y_avg = 13.2427;
z_avg = 5.3357;
rollRaw = 0;
rollAccRaw = 0;

% Initialize fusion algorithm
roll = 0;
pitch = 0;
alpha = 0.9; % 0<=alpha<=1

% Initialize figure
figure('units','normalized','outerposition',[0 0 1 1])
rollR = animatedline('Color','b','Marker','.','MaximumNumPoints',1500);
hold on
rollAcc = animatedline('Color','g','Marker','.','MaximumNumPoints',1500);
hold on
rollF = animatedline('Color','r','Marker','.','MaximumNumPoints',1500);
grid on
title('Roll comparison');
legend('raw roll','roll from accelerometer','fused data');
xlabel('time (s)');
ylabel('degree (°)');


% Read
y = char.empty;
t = 0;
while true
    y = fgets(arduino);
    temp = str2num(y);
    acc = temp(1,:);
    gyr = temp(2,:);
    mag = temp(3,:)-[x_avg,y_avg,z_avg];
    
    % raw roll angle
    rollRaw = gyr(1)*T + rollRaw;
    
    % roll angle from accelerometer
    rollAccRaw = atan2(acc(2),sqrt(acc(1)^2+acc(3)^2))*(180/pi); 
    
    % roll angle from fusion algorithm
    [roll,pitch,yaw] = fusionAccGyr(acc,gyr,mag,roll,pitch,alpha,T);
    
    % display
    addpoints(rollR,t,rollRaw);
    addpoints(rollAcc,t,rollAccRaw);
    addpoints(rollF,t,roll);
    drawnow limitrate
    
    t = t + T;
end