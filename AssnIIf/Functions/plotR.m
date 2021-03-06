function plotR(R,az,el)
% plotR(R,titel);
% plots orientation matrix
%
% R     3x3 matrix
%
% Martin Schepers, December 2005
% Henk Kortier, Oktober 2010

if nargin<2, az=135; end
if nargin<3, el=14; end

figure;
clf; hold on; grid on;
axis image; view([az,el]);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

plot3(...
    [0 R(1,1); ...
     0 R(1,2); ...
     0 R(1,3)]', ...
    [0 R(2,1); ...
     0 R(2,2); ...
     0 R(2,3)]', ...
    [0 R(3,1); ...
     0 R(3,2); ...
     0 R(3,3)]','LineWidth',4);

axis([-1 1 -1 1 -1 1])