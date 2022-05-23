function Rgs=initOrient(acc_s, iflat)
% Rgs=INITORIENT(acc_s, iflat)
%   Calculates an orientation using the assumptions:
%   - Subject is standing still
%   - x-axis of subject body is in the XZ plane of the global reference
%     system.
%
%   - without iflat -> first sample is used
%
% y_b   N*3 accelerometer output, expressed in the body coordinate frame. 
% ti    begin times & end times of intervals in which subject is standing
%       still. Each row contains 2 samplenumbers containing the start- and
%       end times of the interval.
% Rgs   [Xs Ys Zs]', where Xs, Ys, and Zs are the origin of global frame
%       expressed in the sensor frame
% Henk Luinge, 8-12-2002
% Martin Schepers, November 2005
% Henk Kortier, March 2010

if nargin == 1
    Nint = 1;
    Zs_acc = mean(acc_s,1);        % get mean of acc in flat interval
elseif nargin==2
    Nint = 1;
    Zs_acc = mean(acc_s(iflat,:));
else
    error('Initorient can only handle two inputs.');
end
   
%%% Construct matri(x)(ces)    
Xs=[1 0 0]';
for i=1:Nint
    Zs = Zs_acc(i,:)./norm(Zs_acc(i,:));
    Ys = cross(Zs,Xs);
    Ys = Ys./norm(Ys);
    
    Xs = cross(Ys,Zs);
    Xs = Xs./norm(Xs);
    Rgs=[Xs; Ys; Zs];
end

