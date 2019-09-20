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
load('mag_calib_values');
gyrList = zeros([0 3]);
accList = zeros([0 3]);

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.7 0.7])
rO = animatedline('Color','r','MaximumNumPoints',1000);
hold on
pI = animatedline('Color','g','MaximumNumPoints',1000);
hold on
yA = animatedline('Color','b','MaximumNumPoints',1000);
grid on
title('Madgwick algorithm HM');
legend('roll','pitch','yaw');
xlabel('time (s)');
ylabel('degree (°)');

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

% Global variables definition
q0 = [1 0 0 0];           % estimated orientation quaternion elements with initial conditions
b_x = 1; b_z = 0;         % reference direction of flux in earth frame
result = [q0, b_x, b_z];  % result vector structure
beta = 0.8;
zeta = 0.05;
t = 0;
update = clock;

% Read and fuse
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gyr = temp(2,:) - gyrOff;
        mag = temp(3,:)-[x_avg,y_avg,z_avg];

        % Process sensor data through algorithm
        for ii = 1:2
            deltat = etime(clock,update);                                   % calclate deltat
            update = clock;                                                 % initialize next deltat
            %q = fusionMadgwick(q0, acc, gyr*(pi/180), mag, beta, deltat);	% gyroscope units must be radians
            result = fusionMadgwickOriginal(result, acc, gyr*(pi/180), mag, beta, zeta, deltat);	% gyroscope units must be radians
            q = result(1:4);
            euler = quatern2eulerOriginal(q)*(180/pi); 

            % display
            t = t + deltat;
            addpoints(rO,t,euler(1));
            addpoints(pI,t,euler(2));
            addpoints(yA,t,euler(3));
            disp(euler);
            drawnow limitrate
        end
    end
end


%% Read data and 3D
% Initialize data
load('mag_calib_values');
gyrList = zeros([0 3]);
accList = zeros([0 3]);

% Initialize filter
beta = 0.1; %0.2

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
clf
xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);
view(3);
grid on
pbaspect([1 1 1]);

p(1) = patch('YData',[-0.85,-0.85,-0.85,-0.85,-0.85],'XData',[1.75,1.75,0.25,-1.75,-1.75],'ZData',[0,0.4,1.4,1.4,0], 'FaceColor', 'b','FaceAlpha',1);
p(2) = patch('YData',[0.85,0.85,0.85,0.85,0.85],'XData',[1.75,1.75,0.25,-1.75,-1.75],'ZData',[0,0.4,1.4,1.4,0], 'FaceColor', 'b','FaceAlpha',1);
p(3) = patch('YData',[-0.85,-0.85,0.85,0.85],'XData',[1.75,1.75,1.75,1.75],'ZData',[0,0.4,0.4,0], 'FaceColor', 'r','FaceAlpha',1);
p(4) = patch('YData',[-0.85,-0.85,0.85,0.85],'XData',[1.75,0.25,0.25,1.75],'ZData',[0.4,1.4,1.4,0.4], 'FaceColor', 'k','FaceAlpha',.5);
p(5) = patch('YData',[-0.85,-0.85,0.85,0.85],'XData',[0.25,-1.75,-1.75,0.25],'ZData',[1.4,1.4,1.4,1.4], 'FaceColor', 'g','FaceAlpha',1);
p(6) = patch('YData',[-0.85,-0.85,0.85,0.85],'XData',[-1.75,-1.75,-1.75,-1.75],'ZData',[0,1.4,1.4,0], 'FaceColor', 'r','FaceAlpha',1);
p(7) = patch('YData',[-0.85,-0.85,0.85,0.85],'XData',[-1.75,+1.75,+1.75,-1.75],'ZData',[0,0,0,0], 'FaceColor', 'y','FaceAlpha',1);

c = hgtransform;
set(p,'Parent',c);

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

% Global variables definition
q0 = [1 0 0 0];           % estimated orientation quaternion elements with initial conditions
b_x = 1; b_z = 0;         % reference direction of flux in earth frame
result = [q0, b_x, b_z];  % result vector structure
beta = 0.4;
zeta = 0.003;
t = 0;
update = clock;

% Read and fuse
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        acc = (temp(1,:) - [accOff(1),accOff(2),0])./[1,1,accOff(3)];
        gyr = temp(2,:) - gyrOff;
        mag = temp(3,:)-[x_avg,y_avg,z_avg];

        % Process sensor data through algorithm
        for ii = 1:2
            deltat = etime(clock,update);                                   % calclate deltat
            update = clock;                                                 % initialize next deltat
            %q = fusionMadgwick(q0, acc, gyr*(pi/180), mag, beta, deltat);	% gyroscope units must be radians
            result = fusionMadgwickOriginal(result, acc, gyr*(pi/180), mag, beta, zeta, deltat);	% gyroscope units must be radians
            q = result(1:4);
            euler = quatern2eulerOriginal(q);     

            % display
            t = t + deltat;
            c.Matrix = makehgtform('xrotate',euler(1),'yrotate',euler(2),'zrotate',euler(3));
            disp(euler.*(180/pi));
            drawnow limitrate
        end
    end
end