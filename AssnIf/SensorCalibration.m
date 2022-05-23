%% This file is meant for the University of Twente course Biomechatronics.
% Copyright Bouke Scheltinga 2021, BSS University of Twente.

%% Initialize
% clc
clearvars
close all
set(0,'defaultfigurewindowstyle','docked'); % for docked figures ('normal' is default)
filename = 'Your_recording.csv'; % Change for 3.1.1

%% Read Recorded Log
data_raw = readmatrix(filename); % Use for MATLAB version 2019a and higher
% data_raw = readtable(filename); % Use for MATLAB version 2018b and lower
% data_raw = table2array(data_raw); % Use for MATLAB version 2018b and lower

idx = find(data_raw(:,1)==0,1,'last'); % Remove the zeros from the measurement
if isempty(idx)
    idx = 1;
end

data.acc = data_raw(idx+1:end,2:4); % In g
fs = 250; % Sampling frequency (Hz)
ts = 1/fs;
N = length(data.acc);

%% Filter data
f_co = 15; % cut-off, 15 Hz
[Bf, Af] = butter(2, (2 * f_co / fs));% 2nd order low-pass butterworth filter
data.acc = filtfilt(Bf, Af, data.acc);

%% Plot data
norm = sqrt(data.acc(:,1).^2 + data.acc(:,2).^2 + data.acc(:,3).^2);

figure();
subplot(2,1,1)
plot(data.acc); 
ylabel('Acceleration (g)'); 
legend('X^s','Y^s','Z^s'); 
title('Accelerometer - filtered');

subplot(2,1,2)
plot(norm)
ylabel('Norm of accleration (g)'); xlabel('Time (samples)');
title('Norm of accelerometer');

%% Identify the 6-orientations
% e.g. x_pos_start refers to the first sample of the measurement where the x-axis 
% should measure a constant +1g. x_pos_neg refers to the first sample where
% the x-axis should measure -1g.
x_pos_start = 0;    x_pos_end = x_pos_start+250;     % Change this part for assignment 3.1.1
x_neg_start = 0;    x_neg_end = x_neg_start+250;     % Change this part for assignment 3.1.1
y_pos_start = 0;    y_pos_end = y_pos_start+250;     % Change this part for assignment 3.1.1
y_neg_start = 0;    y_neg_end = y_neg_start+250;     % Change this part for assignment 3.1.1
z_pos_start = 0;    z_pos_end = z_pos_start+250;     % Change this part for assignment 3.1.1
z_neg_start = 0;    z_neg_end = z_neg_start+250;     % Change this part for assignment 3.1.1

%% Apply a gain calibration
% Get the mean of the accelerations at the 6 orientations
x_pos = mean(data.acc(x_pos_start:x_pos_end,1));
x_neg = mean(data.acc(x_neg_start:x_neg_end,1)); 
y_pos = mean(data.acc(y_pos_start:y_pos_end,2)); 
y_neg = mean(data.acc(y_neg_start:y_neg_end,2)); 
z_pos = mean(data.acc(z_pos_start:z_pos_end,3));
z_neg = mean(data.acc(z_neg_start:z_neg_end,3)); 

% Calculate gain
x_gain = 0; % Change this part for assignment 3.1.2
y_gain = 0; % Change this part for assignment 3.1.2
z_gain = 0; % Change this part for assignment 3.1.2

% Correct gain
data.acc(:,1) = data.acc(:,1)*x_gain;
data.acc(:,2) = data.acc(:,2)*y_gain;
data.acc(:,3) = data.acc(:,3)*z_gain;

%% Apply offset calibration
% The offset will be determined using the parts of the recording where +1g 
% should have been the measurement outcome.
x_pos = mean(data.acc(x_pos_start:x_pos_end,1));
y_pos = mean(data.acc(y_pos_start:y_pos_end,2));
z_pos = mean(data.acc(z_pos_start:z_pos_end,3));

% Calculate offset
x_offset = x_pos - 0; % Change this part for assignment 3.1.2
y_offset = y_pos - 0; % Change this part for assignment 3.1.2
z_offset = z_pos - 0; % Change this part for assignment 3.1.2

% Correct offset
data.acc(:,1) = data.acc(:,1) - x_offset; % Change this part for assignment 3.1.2
data.acc(:,2) = data.acc(:,2) - y_offset;
data.acc(:,3) = data.acc(:,3) - z_offset;

%% Plot the signals again to check the result of the calibration
norm_calibrated = sqrt(data.acc(:,1).^2 + data.acc(:,2).^2 + data.acc(:,3).^2);

figure();
subplot(2,1,1)
plot(data.acc); 
ylabel('Acceleration (g)'); 
legend('X^s','Y^s','Z^s'); 
title('Accelerometer - filtered and calibrated');

subplot(2,1,2)
plot(norm_calibrated)
ylabel('Norm of accleration (g)'); xlabel('Time (samples)');
title('Norm of calibrated accelerometer');

%% Save the calibration values
sensor_calibration = [x_gain, x_offset; y_gain, y_offset; z_gain, z_offset]; % [x-gain, x-offset; y-gain, y-offset; z-gain, z-offset]
save('Sensor_calibration_values.mat', 'sensor_calibration')