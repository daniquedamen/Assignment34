%% This file is meant for the University of Twente course Biomechatronics.
% Copyright Irfan Refai 2020, BSS University of Twente.
%% Initialize
% clc
clearvars
close all
set(0,'defaultfigurewindowstyle','docked');% for docked figures ('normal' is default)
addpath(genpath('Functions'))

%% Read recorded log
file = 'your_filename.csv'; % Change this name for assignment 3.4.1b
load 'Sensor_calibration_values.mat'; % Move this file to the right folder
data_raw = readmatrix(file);
idx = find(data_raw(:,1)==0,1,'last');
PelvisIMU.acc = data_raw(idx+1:end,2:4); % To m/s

% Use calibration to correct data:
PelvisIMU.acc = PelvisIMU.acc.*sensor_calibration(:,1)' - sensor_calibration(:,2)';
PelvisIMU.acc = PelvisIMU.acc*9.81; % To m/s^2
PelvisIMU.gyr = data_raw(idx+1:end,5:7)*(pi/180); % To rad/s
PelvisIMU.fs = 250; 
PelvisIMU.N = length(PelvisIMU.acc);
clear data_double

%% Section 1: Estimating the pre-defined Rps
figure; 
subplot(211); plot(PelvisIMU.gyr);
ylabel('Angular Velocity (rad/s)');xlabel('Samples'); legend('X^s','Y^s','Z^s'); title('Gyroscope');
subplot(212); plot(PelvisIMU.acc);
ylabel('Acceleration (m^2/s)');xlabel('Samples'); legend('X^s','Y^s','Z^s'); title('Accelerometer');
%% 
% b) Identify static pose, bowing and walking.
stStat = 0;    endStat = 0;     % Change this part for assignment 3.4 
stBow = 0;   endBow = 0;        % Change this part for assignment 3.4 
stWalk = 0; endWalk = 0;        % Change this part for assignment 3.4
Nmov = endWalk - stWalk + 1;

PelvisGYRWalk = (PelvisIMU.gyr(stWalk:endWalk,:)); % walking data
PelvisACCWalk = (PelvisIMU.acc(stWalk:endWalk,:)); 

figure; plot(PelvisGYRWalk);
ylabel('Angular Velocity (rad/s)');xlabel('Samples'); legend('X^s','Y^s','Z^s'); title('Walking Subset');

PelvisGYRBow = (PelvisIMU.gyr(stBow:endBow,:));    % bowing/calibration data
PelvisGYRBowF  = butterfilterlpf(PelvisGYRBow,3,PelvisIMU.fs,2);
figure; plot(PelvisGYRBowF);
ylabel('Angular Velocity (rad/s)');xlabel('Samples'); legend('X^s','Y^s','Z^s'); title('Bowing Subset');

%% Use PCA to estimate Y axis or MedioLateral Axis or Pitch or Pelvis frame
% c) Apply PCA
% Apply PCA on PelvisGYRBowF
wcoeff = 0;         % Change this part for assignment 3.4

% Select the principal component with largest variance
mlEstimate = 0; % Change this part for assignment 3.4

% Select static acceleration to determine inclination due to gravity
PelvisACCStat = 0; % Change this part for assignment 3.4

% d) Edit initOrient_Pelvis to obtain Rinit
Rinit = initOrient_Pelvis(PelvisACCStat,mlEstimate);
RpsHat = intOmega(Rinit,PelvisGYRWalk,1/PelvisIMU.fs); % Only walking data is considered
inclinationDriftCompensation;

%% Transform data to psi_p and verify if the orientation aligns with the definitions in figure 3
% e)
for i = 1:Nmov
    PelvisACCWalk_p(i,:) = 0; % Change this part for assignment 3.4. 
    PelvisGYRWalk_p(i,:) = 0; % Change this part for assignment 3.4. 
end
figure; plot(PelvisACCWalk_p)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Section 2: Applying a Madgwick filter
addpath('quaternion_library');   

% a) Convert the initial Rotation matrix, Rinit, to quaternion form
initQuaternion = 0; % Change this part for assignment 3.4. 

% b) select 3 different values
betaValue = 0; % Change this part for assignment 3.4. 

AHRS = MadgwickAHRS('SamplePeriod', 1/PelvisIMU.fs, 'Beta', betaValue);
quaternion = zeros(length(PelvisACCWalk), 4);
AHRS.Quaternion = initQuaternion; 
quaternion(1,:) = initQuaternion; 

% Apply Madgwick Filter
for t = 2:length(PelvisACCWalk)
    AHRS.UpdateIMU(PelvisGYRWalk(t,:), PelvisACCWalk(t,:));
    quaternion(t, :) = AHRS.Quaternion;
end

% Convert each quaternion to a rotation matrix. 
RpsHatMadgwick = zeros(length(quaternion),3,3);
for iCt = 1:length(quaternion)
    RpsHatMadgwick(iCt,:,:) = 0; % Change this part for assignment 3.4. 
end

%% Transform data using RpsHatMadgwick and compare with PelvisACCWalk_p/PelvisGYRWalk_p
% b) Change this part for assignment 3.4. 
 
for i = 1:Nmov
    PelvisACCWalk_pm(i,:) = 0; % Change this part for assignment 3.4. 
    PelvisGYRWalk_pm(i,:) = 0; % Change this part for assignment 3.4. 
end
figure; plot(PelvisACCWalk_pm) % Plot three instances with varying beta values.

%% End of script