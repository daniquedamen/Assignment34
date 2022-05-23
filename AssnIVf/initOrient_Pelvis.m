function Rgs=initOrient_Pelvis(acc_s, MLEstimate)
% Rgs=INITORIENT(acc_s, iflat)
%   Calculates an orientation using the assumptions:
%   - Subject stood still (Z axis)
%   - Subject performed bowing calibration (Y axis).
%
% Rgs   [Xs Ys Zs]', where Xs, Ys, and Zs are the origin of global frame
%       expressed in the sensor frame
% Henk Luinge, 8-12-2002
% Martin Schepers, November 2005
% Henk Kortier, March 2010
% Irfan Refai, April 2020
% Bouke Scheltinga, April 2022


Zs_acc = mean(acc_s,1);
Ys = MLEstimate';

% Determine unit vector for Z axis
Zs = 0; % Change this part for assignment 3.4.1d. 

% Determine unit vector of Y axis
Ys = 0;         % Change this part for assignment 3.4.1d.  

% Determine X axis from Y and Z
Xs = 0;         % Change this part for assignment 3.4.1d.  

% Scale to unit length
Xs = 0;         % Change this part for assignment 3.4.1d. 

% Make sure your system is orthogonal: determine Y axis from Z and X
Ys = 0;         % Change this part for assignment 3.4.1d. 

Rgs=[Xs; Ys; Zs];


