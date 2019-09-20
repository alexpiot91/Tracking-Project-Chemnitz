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


%% Frequency analysis
% % Initialize data
% load('mag_calib_values');
% data = zeros([0 0]);
% 
% % Initialize fusion algorithm
% roll = 0;
% pitch = 0;
% alpha = 0.9; % 0<=alpha<=1
% 
% % Read
% y = char.empty;
% t = 0;
% while t<12
%     y = fgets(arduino);
%     temp = str2num(y);
%     acc = temp(1,:);
%     gyr = temp(2,:);
%     mag = temp(3,:)-[x_avg,y_avg,z_avg];
%     
%     % Euler angle from fusion algorithm
%     [roll,pitch,yaw] = fusionAccGyr(acc,gyr,mag,roll,pitch,alpha,T);
%     if (t>2)
%         data = cat(1,data,yaw);
%     end
%     
%     t = t + T;
% end
% 
% % Fourier analysis
% Y = fft(data);
% L = length(data);     % Length of signal
% t = (0:L-1)*T;       % Time vector
% P2 = abs(Y/L);
% if mod(L,2) == 1
%     P1 = P2(1:floor(L/2)+1);
% else
%     P1 = P2(1:L/2);
% end
% P1(2:end-1) = 2*P1(2:end-1);
% f = fs*(0:(L/2))/L;
% figure()
% plot(f,P1) ;
% title('Single-Sided Amplitude Spectrum of yaw(t)');
% xlabel('f (Hz)');
% ylabel('|P1(f)|');


%% Read data and 2D NON FILT
% % Initialize data
% load('mag_calib_values');
%      
% % Initialize fusion algorithm
% gyrOff = zeros([0 3]);
% roll = 0;
% pitch = 0;
% alpha = 0.9; % 0<=alpha<=1
% mu = 0.1;
% 
% % Initialize figure
% figure('units','normalized','outerposition',[0 0 1 1])
% rO = animatedline('Color','r','Marker','.','MaximumNumPoints',1000);
% hold on
% pI = animatedline('Color','g','Marker','.','MaximumNumPoints',1000);
% hold on
% yA = animatedline('Color','b','Marker','.','MaximumNumPoints',1000);
% grid on
% title('Complementary Filter');
% legend('roll','pitch','yaw');
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
% update = clock;
% while t<1
%     y = fgets(arduino);
%     temp = str2num(y);
%     size_temp = size(temp);
%     if (size_temp >= [3,3])
%         acc = temp(1,:);
%         gyr = temp(2,:);
%         mag = temp(3,:)-[x_avg,y_avg,z_avg];
% 
%         % Euler angle from fusion algorithm
%         deltat = etime(clock,update);    % calclate deltat
%         update = clock;
%         [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,deltat);
%         gyrOff = cat(1,gyrOff,[roll,pitch,yaw]);
%     end 
%     t = t + deltat;
% end
% 
% % Read and treat
% gyrOff = mean(gyrOff);
% t = 0;
% update = clock;
% while true
%     y = fgets(arduino);
%     temp = str2num(y);
%     size_temp = size(temp);
%     if (size_temp >= [3,3])
%         acc = temp(1,:);
%         gyr = temp(2,:);
%         mag = temp(3,:)-[x_avg,y_avg,z_avg];
% 
%         % Euler angle from fusion algorithm
%         deltat = etime(clock,update);    % calclate deltat
%         update = clock;
%         [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,deltat);
%         euler = [roll,pitch,yaw];
%         euler = euler - [gyrOff(1),gyrOff(2),0];
% 
%         % display
%         addpoints(rO,t,euler(1));
%         addpoints(pI,t,-euler(2));
%         addpoints(yA,t,euler(3));
%         disp(euler);
%         drawnow limitrate
%     end
%     t = t + deltat;
% end


%% Read data and 2D FILT
% % Initialize data
% load('mag_calib_values');
%      
% % Initialize fusion algorithm
% gyrOff = zeros([0 3]);
% yawFilt = zeros([0 0]);
% roll = 0;
% pitch = 0;
% yawF = zeros([0 0]);
% alpha = 0.9; % 0<=alpha<=1
% 
% % Initialize filter
% fc = 1.5;
% [b,a] = butter_synth(1,fc,fs);
% 
% % Initialize figure
% figure('units','normalized','outerposition',[0 0 1 1])
% rO = animatedline('Color','r','MaximumNumPoints',1000);
% hold on
% pI = animatedline('Color','g','MaximumNumPoints',1000);
% hold on
% yA = animatedline('Color','b','MaximumNumPoints',1000);
% grid on
% title('Complementary Filter');
% legend('roll','pitch','yaw');
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
% update = clock;
% while t<1
%     y = fgets(arduino);
%     temp = str2num(y);
%     size_temp = size(temp);
%     if (size_temp >= [3,3])
%         acc = temp(1,:);
%         gyr = temp(2,:);
%         mag = temp(3,:)-[x_avg,y_avg,z_avg];
% 
%         % Euler angle from fusion algorithm
%         deltat = etime(clock,update);    % calclate deltat
%         update = clock;
%         [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,deltat);
%         gyrOff = cat(1,gyrOff,[roll,pitch,yaw]);
%         yawF = cat(1,yawF,yaw);
%     end
%     t = t + deltat;
% end
% 
% % Read and treat
% yawF(1:end-10) = [];
% gyrOff = mean(gyrOff);
% t = 0;
% update = clock;
% while true
%     y = fgets(arduino);
%     temp = str2num(y);
%     size_temp = size(temp);
%     if (size_temp >= [3,3])
%         acc = temp(1,:);
%         gyr = temp(2,:);
%         mag = temp(3,:)-[x_avg,y_avg,z_avg];
% 
%         % Euler angle from fusion algorithm
%         deltat = etime(clock,update);    % calclate deltat
%         update = clock;
%         [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,deltat);
%         euler = [roll,pitch,yaw];
%         euler = euler - [gyrOff(1),gyrOff(2),0];
%         yawF = cat(1,yawF,euler(3));
%         
%         % Filter
%         yawFF = filtfilt(b, a, yawF);
%         yawF(1) = [];
% 
%         % display
%         addpoints(rO,t,euler(1));
%         addpoints(pI,t,-euler(2));
%         addpoints(yA,t,yawFF(end));
%         disp(yawFF);
%         drawnow limitrate
%     end
%     t = t + deltat;
% end


%% Read data and 3D
% Initialize data
load('mag_calib_values');
     
% Initialize fusion algorithm
gyrOff = zeros([0 3]);
yawFilt = zeros([0 0]);
roll = 0;
pitch = 0;
alpha = 0.9; % 0<=alpha<=1
mu = 0.1;

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

% Calculation of the gyroscope offset
t = 0;
while t<1
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp)==[3,3])
        acc = temp(1,:);
        gyr = temp(2,:);
        mag = temp(3,:)-[x_avg,y_avg,z_avg];
    
        % Euler angle from fusion algorithm
        [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,T);
        gyrOff = cat(1,gyrOff,[roll,pitch,yaw]);
        t = t + T;
    end
end

% Read and treat
t = 0;
gyrOff = mean(gyrOff);
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        acc = temp(1,:);
        gyr = temp(2,:);
        mag = temp(3,:)-[x_avg,y_avg,z_avg];

        % Euler angle from fusion algorithm
        [roll,pitch,yaw] = fusionComp(acc,gyr,mag,roll,pitch,alpha,T);
        euler = [roll,pitch,yaw];
        euler = euler - [gyrOff(1),gyrOff(2),0];

        % display
        c.Matrix = makehgtform('xrotate',-euler(1)*pi/180,'yrotate',euler(2)*pi/180,'zrotate',euler(3)*pi/180);
        disp(euler);
        drawnow limitrate
    end
end