function [Rgs] = intOmega(Rgs_init, omega, t_sample)
% intOmega.m - Calculates orientation matrix by integrating gyroscope signal
%
% Inputs: 
% Rgs_init          Initial orientation matrix
% omega             Angular velocity (rad/s)
% t_sample          Sample time
%
% Outputs: 
% Rgs               Orientation matrix [Nsamples x 3 x 3]
%
% Martin Schepers, July 2005
% Henk Kortier, Sep 2011

N = size(omega,1);               
omega_sg_s_tilde = zeros(N,3,3);

for i=1:N
    omega_sg_s_tilde(i,:,:) = [0 -omega(i,3) omega(i,2); 
                               omega(i,3) 0 -omega(i,1);
                               -omega(i,2) omega(i,1) 0];
end

Rgs = zeros(N,3,3);
Rgs(1,:,:) = Rgs_init;  % initial value
        
for i=2:N
    R = squeeze(Rgs(i-1, :, :));
    
    Rdot = R*squeeze(omega_sg_s_tilde(i,:,:));   % Integrate R
    R = R + t_sample.*Rdot;
    
    [A,~]=qr(R);                                 % create proper rotation matrix
    Rgs(i,:,:) = sqrt(A.*A).*sign(R);            % get correct sign
end