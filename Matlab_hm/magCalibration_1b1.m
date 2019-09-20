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
fc = 1.5;  % cut off frequency for LPF

y = char.empty;
while strcmp(y,char([71,79,13,10])) ~= 1
    y = char(fscanf(arduino));
    disp(y);
end


%% Magnetometer XY calibration
% Initialize figure
figure()
l = animatedline('Color','r','Marker','.');

% Initialize data
mag = zeros([0 0]);
ms = 0;

% Data acquisition
disp('==Rotate the device around the Z-axis==');
t0 = clock;
while ms < 10 
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp) == [3,3])
        mag = cat(1,mag,temp(3,1:2));
        addpoints(l,mag(end,1),mag(end,2));
        drawnow limitrate
        ms = etime(clock,t0);
    end
end
disp('==STOP==');

% % Fourier analysis
% Y = fft(mag);
% L = length(mag);     % Length of signal
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
% title('Single-Sided Amplitude Spectrum of B(t)');
% xlabel('f (Hz)');
% ylabel('|P1(f)|');

% Filter
[b,a] = butter_synth(1,fc,fs);  % from mytoolbox
magB = filter(b,a,mag);         % built-in function

magF = magB;
for i=1:50
    magF(1,:)=[];
end

scalex1 = max(magF(:,1))-min(magF(:,1));
scaley1 = max(magF(:,2))-min(magF(:,2));
x1_avg = (max(magF(:,1))+min(magF(:,1)))/2;
y1_avg = (max(magF(:,2))+min(magF(:,2)))/2;


figure()
plot(mag(:,1),mag(:,2),'Color','r');
hold on
plot(magF(:,1),magF(:,2),'Color','b');
hold on
plot(magF(:,1)-x1_avg,magF(:,2)-y1_avg,'Color','g');
title('XY Calibration')
xlabel('m_x (G)');
ylabel('m_y (G)');
legend('raw data','after LP filtration','calibrated data');
axis equal
grid on


%% Magnetometer YZ calibration
% Initialize figure
figure()
l = animatedline('Color','r','Marker','.');

% Initialize data
mag = zeros([0 0]);

% Read
ms = 0;
t0 = clock;
while ms < 4
    y = fgets(arduino);
    disp(y);
    ms = etime(clock,t0);
end

disp('==Rotate the device around the X-axis==');
y = char.empty;
ms = 0;
t0 = clock;
while ms < 10 
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp) == [3,3])
        mag = cat(1,mag,temp(3,2:3));
        addpoints(l,mag(end,1),mag(end,2));
        drawnow limitrate
        ms = etime(clock,t0);
    end
end
disp('==STOP==');

% Filter
[b,a] = butter_synth(1,fc,fs);  % from mytoolbox
magB = filter(b,a,mag);         % built-in function

magF = magB;
for i=1:50
    magF(1,:)=[];
end

scaley2 = max(magF(:,1))-min(magF(:,1));
scalez2 = max(magF(:,2))-min(magF(:,2));
y2_avg = (max(magF(:,1))+min(magF(:,1)))/2;
z2_avg = (max(magF(:,2))+min(magF(:,2)))/2;

figure()
plot(mag(:,1),mag(:,2),'Color','r');
hold on
plot(magF(:,1),magF(:,2),'Color','b');
hold on
plot(magF(:,1)-y2_avg,magF(:,2)-z2_avg,'Color','g');
title('YZ Calibration')
xlabel('m_y (G)');
ylabel('m_z (G)');
legend('raw data','after LP filtration','calibrated data');
axis equal
grid on


%% Magnetometer XZ calibration
% Initialize figure
figure()
m = animatedline('Color','r','Marker','.');

% Initialize data
mag = zeros([0 0]);

% Read
y = char.empty;
ms = 0;
t0 = clock;
while ms < 4
    y = fgets(arduino);
    ms = etime(clock,t0);
end

disp('==Rotate the device around the Y-axis==');
y = char.empty;
ms = 0;
t0 = clock;
while ms < 10 
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp) == [3,3])
        mag = cat(1,mag,temp(3,[1 3]));
        addpoints(m,mag(end,1),mag(end,2));
        drawnow limitrate
        ms = etime(clock,t0);
    end
end
disp('==STOP==');

% Filter
[b,a] = butter_synth(1,fc,fs);  % from mytoolbox
magB = filter(b,a,mag);         % built-in function

magF = magB;
for i=1:50
    magF(1,:)=[];
end

scalex3 = max(magF(:,1))-min(magF(:,1));
scalez3 = max(magF(:,2))-min(magF(:,2));
x3_avg = (max(magF(:,1))+min(magF(:,1)))/2;
z3_avg = (max(magF(:,2))+min(magF(:,2)))/2;


figure()
plot(mag(:,1),mag(:,2),'Color','r');
hold on
plot(magF(:,1),magF(:,2),'Color','b');
hold on
plot(magF(:,1)-x3_avg,magF(:,2)-z3_avg,'Color','g');
title('XZ Calibration')
xlabel('m_x (G)');
ylabel('m_z (G)');
legend('raw data','after LP filtration','calibrated data');
axis equal
grid on


%% Bias calculation
x_avg = max(x1_avg,x3_avg);
y_avg = max(y1_avg,y2_avg);
z_avg = max(z2_avg,z3_avg);
scalex = (scalex1+scalex3)/2;
scaley = (scaley1+scaley2)/2;
scalez = (scalez2+scalez3)/2;
scalex_avg = ((scalex+scaley+scalez)/3)/scalex;
scaley_avg = ((scalex+scaley+scalez)/3)/scaley;
scalez_avg = ((scalex+scaley+scalez)/3)/scalez;

save('mag_calib_values','x_avg','y_avg','z_avg','scalex_avg','scaley_avg','scalez_avg');

%% Test
load('mag_calib_values');

% Initialization figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])

[x,y,z] = sphere(40);
x = x*500; y = y*500; z = z*500;
h = surf(x, y, z);
set(h,'edgecolor',[0.5 0.3 0.5],'EdgeAlpha',0.2,'facecolor',[0.8 0.1 0.6],'FaceAlpha',.07)
hold on

magplot = animatedline('Color','r','Marker','.'); %,'MaximumNumPoints',2000);
xlim([-700 700]);
ylim([-700 700]);
zlim([-700 700]);
xlabel('mag_x (mG)');
ylabel('mag_y (mG)');
zlabel('mag_z (mG)');
view(3);
grid on
pbaspect([1 1 1]);

% Stabilization
y = char.empty;
ms = 0;
t0 = clock;
while ms < 1
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    ms = etime(clock,t0);
end

% Read
while true
    y = fgets(arduino);
    temp = str2num(y);
    if (size(temp)==[3,3])
        mag = temp(3,:);
        %mag = offset_scale(mag,x_avg,y_avg,z_avg,scalex_avg,scaley_avg,scalez_avg);
        addpoints(magplot,mag(1),mag(2),mag(3));
        disp(mag);
        drawnow limitrate
    end
end
disp('==STOP==');