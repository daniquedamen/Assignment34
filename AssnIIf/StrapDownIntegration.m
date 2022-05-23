%% This file is meant for the University of Twente course Biomechatronics.
% Copyright Frank Wouda & Dirk Weenk 2016, BSS University of Twente.

%% Initialize
clc
clearvars
close all
set(0,'defaultfigurewindowstyle','docked'); % for docked figures ('normal' is default)
addpath(genpath('Functions'))
self_collected_data = false; % Use false if prerecorded data is loaded (3.2.1 - 3.2.4) 
                            % and true if the data is collected yourself
                            % with the homekit (3.2.5)
%% Read data 
% Read recorded datalog for question 3.2.1 - 3.2.4
if self_collected_data == false
    file = 'short.txt';
    data = readCalibratedMTwLog(file); % for reading MTw files
    fs = data.fs;
    ts = 1/fs;
    N = data.N;
    
    % Self collected data for question 3.2.5
elseif self_collected_data == true
    filename = 'your_filename.csv'     % Change this name for assignment 3.2.5
    load 'Sensor_calibration_values.mat'
    data_raw = readmatrix(filename); % Use for MATLAB version 2019a and higher
    % data_raw = readtable(filename); % Use for MATLAB version 2018b and lower
    % data_raw = table2array(data_raw); % Use for MATLAB version 2018b and lower
    idx = find(data_raw(:,1)==0,1,'last');
    if isempty(idx)
        idx = 1;
    end
    
    data.acc = data_raw(idx+1:end,2:4); % To m/s
    % Use calibration to correct data:
    data.acc = data.acc.*sensor_calibration(:,1)' - sensor_calibration(:,2)';
    data.acc = data.acc*9.81; % To m/s^2
    data.gyr = data_raw(idx+1:end,5:7)*(pi/180); % To rad/s
    fs = 250;
    ts = 1/fs;
    N = length(data.acc);
end

%% Filter data
f_co = 30; % cut-off, 30 Hz
[Bf, Af] = butter(2, (2 * f_co / fs));% 2nd order low-pass butterworth filter
data.acc = filtfilt(Bf, Af, data.acc);
data.gyr = filtfilt(Bf, Af, data.gyr);

%% Plot filtered data
figure();
subplot(2,1,1);
plot(data.acc); 
ylabel('Acceleration (m/s^2)'); legend('X^s','Y^s','Z^s'); title('Accelerometer - filtered');

subplot(2,1,2);
plot(data.gyr);
ylabel('Angular Velocity (rad/s)'); xlabel('Time (s)'); legend('X^s','Y^s','Z^s'); title('Gyroscope - filtered');

%% Start / Stop of movement
if self_collected_data == false
    thGyr = 0.01; % threshold gyroscope signal
    gyrRes = sqrt(sum(data.gyr.^2,2));% magnitude
    Tstart = find(gyrRes > thGyr*max(gyrRes),1,'first')-10;
    Tend = find(gyrRes > thGyr*max(gyrRes),1,'last')+10;
    Nmov = Tend-Tstart+1;
    
elseif self_collected_data == true
    Tstart = 0;       % Set start point of zero velocity update 3.2.5
    Tend = 0;         % Set end point of zero velocity update 3.2.5
    Nmov = Tend-Tstart+1;
end
%% Gyro bias detrend 
gyrOffsetB = mean(data.gyr(Tstart:Tstart+10,:));
gyrOffsetE = mean(data.gyr(Tend-10:Tend,:));

gyrOffMat = [linspace(gyrOffsetB(1),gyrOffsetE(1),N)'...
             linspace(gyrOffsetB(2),gyrOffsetE(2),N)'...
             linspace(gyrOffsetB(3),gyrOffsetE(3),N)'];

data.gyr = data.gyr-gyrOffMat;
%% Initial orientation
Rinit = initOrient(data.acc(Tstart,:));
%Rinit = squeeze(data.R(1,:,:));
%% Integration of angular velocity
RgsHat = intOmega(Rinit,data.gyr(Tstart:Tend,:),ts);
%% Inclination driftcompensation
inclinationDriftCompensation
%% Transformation of acceleration to global frame and substract g
a_g = zeros(Nmov,3); 
g = norm(mean(data.acc(1:100,:)));
for i = 1:Nmov
    a_g(i,:) = squeeze(RgsHat(i,:,:))*data.acc(Tstart+i,:)' + [0 0 -g]';
end





%% Strapdown integration: obtain velocity, should be implemented by you!
% Initialize the velocity vector.
v_g = zeros(Nmov,3); 

for i = 2:Nmov
    v_g(i,:) = 0;                   % Change this part for assignment 3.2.1.
end





%% Plot velocity
time_axis=(0:length(v_g)-1)/fs;
figure;
plot(time_axis,v_g);
xlabel('Time (s)');
ylabel('velocity (m/s)');
legend('x','y','z');
title('Velocity before drift compensation')





%% Velocity drift compensation, should be implemented by you!

for i = 2:Nmov
    v_g(i,:) = v_g(i,:) - 0;        % Change factor 0 to compensate velocity drift (3.2.2).
end





%% Plot velocity, after drift compensation
figure;
plot(time_axis,v_g);
xlabel('Time (s)');
ylabel('velocity (m/s)');
legend('x','y','z');
title('Velocity after drift compensation')





%% Strapdown integration: obtain position, should be implemented by you!
% Initialize the position vector.
r_g = zeros(Nmov,3);

for i = 2:Nmov
    r_g(i,:) = 0;                   % Change this part for assignment 3.2.3.
end





%% Plot trajectory
plotTrajectory





%% Vertical position drift compensation, should be implemented by you!

for i = 2:Nmov
    r_g(i,3) = r_g(i,3) - 0;        % Change factor 0 to compensate velocity drift (3.2.4).
end





%% Plot trajectory
plotTrajectory