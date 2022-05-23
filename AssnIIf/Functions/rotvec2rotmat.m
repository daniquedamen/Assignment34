function R = rotvec2rotmat(vec,angle)
% rotvec2rotmat.m - Transformation of rotation vector + angle to rotation
% matrix
%
% Inputs:
% vec         rotation vector
% angle       rotation angle
%
% Output:
% R           rotation matrix
%
% Henk Kortier, Sep 2011

vecTilde = [0 -vec(3) vec(2);vec(3) 0 -vec(1);-vec(2) vec(1) 0];
vecTensProd = [vec(1)^2 vec(1)*vec(2) vec(1)*vec(3);...
               vec(1)*vec(2) vec(2)^2 vec(2)*vec(3);...
               vec(1)*vec(3) vec(2)*vec(3) vec(3)^2];

R = eye(3)*cos(angle)+sin(angle)*vecTilde+(1-cos(angle))*vecTensProd;