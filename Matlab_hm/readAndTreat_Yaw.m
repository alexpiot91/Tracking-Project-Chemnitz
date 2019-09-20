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
    % while y~='GO', stay in the loop
    y = char(fscanf(arduino));
    disp(y);
end


%% Read data and treat yaw flat surface
% % Initialize data
% load('mag_calib_values');
%      
% % Initialize fusion algorithm
% gyrOff = zeros([0 0]);
% yawOld = 0;
% alpha = 0.3; % 0<=alpha<=1
% 
% % Initialize figure
% figure('units','normalized','outerposition',[0 0 1 1])
% yA = animatedline('Color','b','Marker','.','MaximumNumPoints',1000);
% grid on
% title('Yaw calculation on flat surface');
% legend('yaw');
% xlabel('time (s)');
% ylabel('degree (°)');
% 
% % Wait for stabilization
% t = 0;
% while t<1
%     y = fgets(arduino);
%     temp = str2num(y);
%     t = t + T;
% end
% 
% % Calculation of the gyroscope offset
% t = 0;
% while t<1
%     y = fgets(arduino);
%     temp = str2num(y);
%     acc = temp(1,:);
%     gyr = temp(2,:);
%     
%     % Euler angle from fusion algorithm
%     yaw = fusionYawFlat(acc,gyr,yawOld,alpha,T);
%     gyrOff = cat(1,gyrOff,yaw);
%     yawOld = yaw;
%     t = t + T;
% end
% 
% % Read and treat
% t = 0;
% gyrOff = mean(gyrOff);
% while true
%     y = fgets(arduino);
%     temp = str2num(y);
%     %if (size(temp)==[3,3])
%         acc = temp(1,:);
%         gyr = temp(2,:);
% 
%         % Euler angle from fusion algorithm
%         yaw = fusionYawFlat(acc,gyr,yawOld,alpha,T);
%         yaw = yaw - gyrOff;
%         yawOld = yaw;
%         
%         % display
%         addpoints(yA,t,yaw);
%         disp(temp);
%         drawnow limitrate
%     %end
%     t = t + T;
% end


%% Car visualization
% Initialize data
load('mag_calib_values');
     
% Initialize fusion algorithm
gyrOff = zeros([0 0]);
yawOld = 0;
alpha = 0.3; % 0<=alpha<=1

% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
clf
xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);
view(3);
grid on
pbaspect([1 1 1]);

p(1) = patch('XData',[-0.85,-0.85,-0.85,-0.85,-0.85],'YData',[1.75,1.75,0.25,-1.75,-1.75],'ZData',[0,0.4,1.4,1.4,0], 'FaceColor', 'b','FaceAlpha',1);
p(2) = patch('XData',[0.85,0.85,0.85,0.85,0.85],'YData',[1.75,1.75,0.25,-1.75,-1.75],'ZData',[0,0.4,1.4,1.4,0], 'FaceColor', 'b','FaceAlpha',1);
p(3) = patch('XData',[-0.85,-0.85,0.85,0.85],'YData',[1.75,1.75,1.75,1.75],'ZData',[0,0.4,0.4,0], 'FaceColor', 'r','FaceAlpha',1);
p(4) = patch('XData',[-0.85,-0.85,0.85,0.85],'YData',[1.75,0.25,0.25,1.75],'ZData',[0.4,1.4,1.4,0.4], 'FaceColor', 'k','FaceAlpha',.5);
p(5) = patch('XData',[-0.85,-0.85,0.85,0.85],'YData',[0.25,-1.75,-1.75,0.25],'ZData',[1.4,1.4,1.4,1.4], 'FaceColor', 'g','FaceAlpha',1);
p(6) = patch('XData',[-0.85,-0.85,0.85,0.85],'YData',[-1.75,-1.75,-1.75,-1.75],'ZData',[0,1.4,1.4,0], 'FaceColor', 'r','FaceAlpha',1);
p(7) = patch('XData',[-0.85,-0.85,0.85,0.85],'YData',[-1.75,+1.75,+1.75,-1.75],'ZData',[0,0,0,0], 'FaceColor', 'y','FaceAlpha',1);

c = hgtransform;
set(p,'Parent',c);


% Wait for stabilization
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    t = t + T;
end

% Calculation of the gyroscope offset
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    acc = temp(1,:);
    gyr = temp(2,:);
    
    % Euler angle from fusion algorithm
    yaw = fusionYawFlat(acc,gyr,yawOld,alpha,T);
    gyrOff = cat(1,gyrOff,yaw);
    yawOld = yaw;
    t = t + T;
end

% Read and treat
t = 0;
gyrOff = mean(gyrOff);
while true
    y = fgets(arduino);
    temp = str2num(y);
    %if (size(temp)==[3,3])
        acc = temp(1,:);
        gyr = temp(2,:);

        % Euler angle from fusion algorithm
        yaw = fusionYawFlat(acc,gyr,yawOld,alpha,T);
        yaw = yaw - gyrOff;
        yawOld = yaw;
        
        % display
        c.Matrix = makehgtform('zrotate',yaw*pi/180);
        disp(temp);
        drawnow limitrate
    %end
    t = t + T;
end