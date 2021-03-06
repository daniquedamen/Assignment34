figure();
clf; hold on; %grid on;
for i = 1:floor(fs/10):Nmov
    R = squeeze(RgsHat(i,:,:))./10;
    rx = r_g(i,1); ry = r_g(i,2); rz = r_g(i,3); 
    set(gca,'ColorOrderIndex',1)
   
    Hgs = [[R [rx ry rz]'];[0 0 0 1]];
    
plot3([Hgs(1,4) (Hgs(1,4) + Hgs(1,1)); ...
       Hgs(1,4) (Hgs(1,4) + Hgs(1,2)); ...
       Hgs(1,4) (Hgs(1,4) + Hgs(1,3))]', ...
      [Hgs(2,4) (Hgs(2,4) + Hgs(2,1)); ...
       Hgs(2,4) (Hgs(2,4) + Hgs(2,2)); ...
       Hgs(2,4) (Hgs(2,4) + Hgs(2,3))]', ...
      [Hgs(3,4) (Hgs(3,4) + Hgs(3,1)); ...
       Hgs(3,4) (Hgs(3,4) + Hgs(3,2)); ...
       Hgs(3,4) (Hgs(3,4) + Hgs(3,3))]',...
       'LineWidth',4);
   
   plot3(rx,ry,rz,'ko','LineWidth',2);
end
axis image; view([123 16]);
title('Trajectory');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
grid on; hold off;