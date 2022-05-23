function [vec angle] = rotmat2rotvec(R)
% rotmat2rotvec.m - Transformation of rotation matrix to rotation vector + angle 
% 
% Input:
% R           rotation matrix
%
% Outputs:
% vec         rotation vector
% angle       rotation angle
%
% Henk Kortier, Sep 2011

angle = acos((trace(R)-1)/2);
vec = 1/(2*sin(angle))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2);];