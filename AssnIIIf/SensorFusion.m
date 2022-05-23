%% This file is meant for the University of Twente course Biomechatronics.
% Copyright Frank Wouda 2016, BSS University of Twente.

clear all; close; clc

%% Loading the correct data in the preferred variable names.
% Specify the filename you want to load, provided that your matlab has the
% correct current folder, edit this if you would like to load a different file.

% The data includes approximately 10 seconds stationary on the table, 10
% seconds of movement through the air and approximately 10 second
% stationary at the original position.
filenam = 'MT_012000C2-006';

% Load the chosen file.
load(filenam)

% Extract the acceleration and gyroscope data, and construct a time vector.
acc = rawdata.acc{1};       % m/s^2
gyr = rawdata.gyr{1};       % deg/s
t = rawdata.time{1};
Fs = rawdata.sampleRate;    % Hz
t = (t - t(1))/Fs;          % s
dt = 1/Fs;                  % s

% Calculate the length of the measurement.
n = length(acc);

% initial estimation covariance
P = 0;

% State vector initialization
xhat = nan(1,n);

% Initial estimate of our states.
xinit = 0;

% Kalman vector initializationa
K = nan(1,n);





%% This part has to be edited by you.
% Calculate the orientation with the accelerometer.
orien_acc = 0;          % Change this part for assignment 3.3.

% Calculate the orientation with the gyroscope.
orien_gyr = 0;          % Change this part for assignment 3.3.

% State space model for sensor fusion.
A = 0;                  % Change this part for assignment 3.3.
B = 0;                  % Change this part for assignment 3.3.
H = 0;                  % Change this part for assignment 3.3.

% Measured value.
y = 0;                  % Change this part for assignment 3.3.

% Input value.
u = 0;                  % Change this part for assignment 3.3.

% Process noise
Q = 0;                  % Change this part for assignment 3.3.

% Measurement noise.
R = 0;                  % Change this part for assignment 3.3.







%% Kalman filter design implemented according to Bishop.
for i = 1:n
    % At the first step no measurement is available yet.
    if i == 1
        xhat(i) = A*xinit;              % Prediction step
    else
        xhat(i) = A*xhat(i-1)+B*u(i-1); % Prediction step
    end
    Pprior = A * P * A' + Q;                            % A priori error covariance
    K(:,i) = (Pprior * H')/(H * Pprior * H' + R);       % Kalman gain
    xhat(i) = xhat(i) + K(:,i) * (y(i) - H * xhat(i));  % Measurement update
    P = (eye(size(xhat,1)) - K(:,i) * H)* Pprior;       % A posterior error covariance
end

%% Plotting commands, you should not change this part!
figure;
plot(t,orien_acc,'LineWidth',2)
hold on;
plot(t,orien_gyr,'LineWidth',2)
plot(t,xhat(1,:),'LineWidth',2)
legend('Accelerometer estimation','Gyroscope estimation','Kalman estimation',...
    'Location','NorthWest')
xlabel('Time (s)')
xlim([t(1) t(end)])
ylabel('Inclination (deg)')
title('Comparison of sensor orientation estimations')