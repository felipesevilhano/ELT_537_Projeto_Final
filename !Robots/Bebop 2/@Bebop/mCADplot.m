function mCADplot(drone,lines)
% Plot Bebop 3D CAD model on its current position
% drone.pPos.X = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

if nargin < 2
    drone.pCAD.flagLines = 0;
else
    drone.pCAD.flagLines = lines;
end

% To creat 'Esqueleto' Model
if drone.pCAD.flagLines == 1  % Create the demon line
    drone.pCAD.i2D.Vertices = [drone.pPos.X(1) drone.pPos.X(1) drone.pPos.X(1)+0.0001;...
    drone.pPos.X(2) drone.pPos.X(2) drone.pPos.X(2);...
    0 drone.pPos.X(3) drone.pPos.X(3)]';
end


% --- Init CAD Model
if drone.pCAD.flagCreated == 0
    drone.pPos.psiHel = 0;
    
    mCADmake(drone)
    mCADplot(drone)
    
    for idx = 1:length(drone.pCAD.i3D)
        drone.pCAD.i3D{idx}.FaceAlpha = 1.0;
        drone.pCAD.i3D{idx}.Visible = 'on';
    end
    
else
    % Update drone pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot drone.pPos.X(1:3); 0 0 0 1];
    
    vertices = H*[drone.pCAD.obj{1}.v; ones(1,size(drone.pCAD.obj{1}.v,2))];
    %drone.pCAD.i3D.Vertices = vertices(1:3,:)';
    
    set(drone.pCAD.i3D{1}, 'Vertices',vertices(1:3,:)');
    
    
    %%% Rotational matrix (Hélice Frente Dir.)
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [0.15-.03; 0.1825-.03; .01]; 0 0 0 1];
    
    
    Rot_J1 = [ cos(drone.pPos.psiHel) sin(drone.pPos.psiHel) 0;
              -sin(drone.pPos.psiHel) cos(drone.pPos.psiHel) 0;
                        0                      0             1];
            
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{2}.v; ones(1,size(drone.pCAD.obj{2}.v,2))];
    drone.pCAD.i3D{2}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Esq.)
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.15+.02; -0.1825+.03; .01]; 0 0 0 1];

                    
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{3}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Frente Esq.)
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [.15-.03; -0.1825+.03; .01]; 0 0 0 1];
    
    Rot_J1 = [ cos(-(drone.pPos.psiHel + pi+0.225)) sin(-(drone.pPos.psiHel + pi+0.225)) 0;
              -sin(-(drone.pPos.psiHel + pi+0.225)) cos(-(drone.pPos.psiHel + pi+0.225)) 0;
                              0                              0               1];
                          
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{2}.v; ones(1,size(drone.pCAD.obj{2}.v,2))];
    drone.pCAD.i3D{4}.Vertices = vertices(1:3,:)';
    
    
    %%% Rotational matrix (Hélice Tras Dir.)
    RotX = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
    RotY = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
    RotZ = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H1 = [Rot [-0.15+.02; 0.1875-.03; .01]; 0 0 0 1];
                        
    vertices = H*H1*[Rot_J1*drone.pCAD.obj{3}.v; ones(1,size(drone.pCAD.obj{3}.v,2))];
    drone.pCAD.i3D{5}.Vertices = vertices(1:3,:)';
   

end

if strcmp(drone.pPar.fightState,'taking-off') || ...
   strcmp(drone.pPar.fightState,'hovering') || ...
   strcmp(drone.pPar.fightState,'navigating')
    drone.pPos.psiHel = drone.pPos.psiHel + (1 + (1.5-1)*rand(1))*2*pi/6;
end
    
    
end


% =========================================================================
function mCADmake(uav)

for i = 1:length(uav.pCAD.obj)
    
    hold on
    uav.pCAD.i3D{i} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
        
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{i}.FaceColor = 'flat';
    uav.pCAD.i3D{i}.EdgeColor = 'none';
    uav.pCAD.i3D{i}.FaceAlpha = 0.0;
    uav.pCAD.i3D{i}.Visible = 'off';
    % light;

end

for iP = 4:5
    
    i = iP-2;
    
    hold on
    uav.pCAD.i3D{iP} = patch('Vertices',uav.pCAD.obj{1,i}.v','Faces',uav.pCAD.obj{1,i}.f3');
    hold off
    
    fvcd3 = [];
    
    for ii = 1:length(uav.pCAD.obj{i}.umat3)
        mtlnum = uav.pCAD.obj{i}.umat3(ii);
        for jj=1:length(uav.pCAD.mtl{i})
            if strcmp(uav.pCAD.mtl{i}(jj).name,uav.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = uav.pCAD.mtl{i}(jj).Kd';
        
    end

    % size(fvcd3)
    
    uav.pCAD.i3D{iP}.FaceVertexCData = fvcd3;
    uav.pCAD.i3D{iP}.FaceColor = 'flat';
    uav.pCAD.i3D{iP}.EdgeColor = 'none';
    uav.pCAD.i3D{iP}.FaceAlpha = 0.0;
    uav.pCAD.i3D{iP}.Visible = 'off';
end
    

uav.pCAD.flagCreated = 1;

drawnow limitrate nocallbacks


end