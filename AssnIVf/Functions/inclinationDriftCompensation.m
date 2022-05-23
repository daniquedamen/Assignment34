RpsHatEnd = squeeze(RpsHat(end,:,:));       % Final orienation
Rdiff = Rinit'*RpsHatEnd;                   % Total orientation difference

theta = atan2(Rdiff(2,1),Rdiff(1,1));       % Angle of orientation in xy plane
RdiffXY = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1]; % R matrix describing Rdiff in xy

Rdrift = Rinit'*(RdiffXY'*RpsHatEnd);       % Orientation describing drift wrt vertical

[vec theta] = rotmat2rotvec(Rdrift);
dtheta = theta/Nmov;

for i = 1:Nmov
    dtheta_i = (i-1)*dtheta;
    Rmat_i = rotvec2rotmat(vec,dtheta_i);
    RpsHat(i,:,:) = squeeze(RpsHat(i,:,:))*Rmat_i';
end