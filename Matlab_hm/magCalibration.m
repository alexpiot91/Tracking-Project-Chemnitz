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
fs = 200;
T = 1/fs;
fc = 1.5;  % cut off frequency for LPF

y = char.empty;
while strcmp(y,char([71,79,13,10])) ~= 1
    y = char(fscanf(arduino));
    disp(y);
end


%% Magnetometer calibration
% Initialize figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
l = animatedline('Color','r','Marker','.');
view(3)
grid on

% Wait for stabilization
t0 = clock;
s = 0;
while s<1
    y = fgets(arduino);
    temp = str2num(y);
    s = etime(clock,t0);
end

% Data acquisition
disp('==Rotate the device around all the axes==');
mag = zeros([0 3]);
s = 0;
t0 = clock;
while s < 30 
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        mag = cat(1,mag,temp(3,:));
        addpoints(l,mag(end,1),mag(end,2),mag(end,3));
        drawnow limitrate
        s = etime(clock,t0);
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

scalex = (max(magF(:,1))-min(magF(:,1)))/2;
scaley = (max(magF(:,2))-min(magF(:,2)))/2;
scalez = (max(magF(:,3))-min(magF(:,3)))/2;
scale_avg = (scalex+scaley+scalez)/3;
scalex_avg = scale_avg/scalex;
scaley_avg = scale_avg/scaley;
scalez_avg = scale_avg/scalez;
x_avg = (max(magF(:,1))+min(magF(:,1)))/2;
y_avg = (max(magF(:,2))+min(magF(:,2)))/2;
z_avg = (max(magF(:,3))+min(magF(:,3)))/2;

save('mag_calib_values','x_avg','y_avg','z_avg','scalex_avg','scaley_avg','scalez_avg');

figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])
plot3(mag(:,1),mag(:,2),mag(:,3),'Color','r');
hold on
plot3(magF(:,1),magF(:,2),magF(:,3),'Color','b');
hold on
plot3((magF(:,1)-x_avg)*scalex_avg,(magF(:,2)-y_avg)*scaley_avg,(magF(:,3)-z_avg)*scalez_avg,'Color','g');
xlabel('m_x (mG)');
ylabel('m_y (mG)');
legend('raw data','after LP filtration','calibrated data');
axis equal
grid on


%% Test
% Initialization figure
figure('units','normalized','outerposition',[0.2 0.2 0.8 0.8])

[x,y,z] = sphere(40);
x = x*scale_avg; y = y*scale_avg; z = z*scale_avg;
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
s = 0;
t0 = clock;
while s < 1
    y = fgets(arduino);
    temp = str2num(y);
    disp(temp);
    s = etime(clock,t0);
end

% Read
while true
    y = fgets(arduino);
    temp = str2num(y);
    size_temp = size(temp);
    if (size_temp >= [3,3])
        mag = temp(3,:);
        mag = offset_scale(mag,x_avg,y_avg,z_avg,scalex_avg,scaley_avg,scalez_avg);
        addpoints(magplot,mag(1),mag(2),mag(3));
        disp(mag);
        drawnow limitrate
    end
end
disp('==STOP==');