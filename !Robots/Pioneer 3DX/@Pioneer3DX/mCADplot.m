function mCADplot(obj)
% Plot Pioneer 3D CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

if nargin < 2
    obj.pCAD.flagLines = 0;
end


if obj.pCAD.flagCreated == 0
    mCADmake(obj)
    mCADplot(obj)

    mCADplot(obj)

    obj.pCAD.i3D{1}.FaceAlpha = 1.0;
    obj.pCAD.i3D{1}.Visible = 'on';
    
    
else
    obj.pCAD.i3D{1}.FaceAlpha = 1;
    obj.pCAD.i3D{1}.Visible = 'on';

    % Update robot pose
    %%% Rotational matrix
    RotX = [1 0 0; 0 cos(obj.pPos.Xc(4)) -sin(obj.pPos.Xc(4)); 0 sin(obj.pPos.Xc(4)) cos(obj.pPos.Xc(4))];
    RotY = [cos(obj.pPos.Xc(5)) 0 sin(obj.pPos.Xc(5)); 0 1 0; -sin(obj.pPos.Xc(5)) 0 cos(obj.pPos.Xc(5))];
    RotZ = [cos(obj.pPos.Xc(6)) -sin(obj.pPos.Xc(6)) 0; sin(obj.pPos.Xc(6)) cos(obj.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    H = [Rot obj.pPos.Xc(1:3); 0 0 0 1];
    
    vertices = H*[obj.pCAD.obj{1}.v; ones(1,size(obj.pCAD.obj{1}.v,2))];
    obj.pCAD.i3D{1}.Vertices = vertices(1:3,:)';

    hold on
    %obj.pCAD.ObjImag(1) = plot3([obj.pPos.Xc(1) obj.pPos.X(1)],[obj.pPos.Xc(2) obj.pPos.X(2)],[0.1 0.1],'-*','Color',[1 1 1]*.5);
    obj.pCAD.ObjImag(1).XData = [obj.pPos.Xc(1) obj.pPos.X(1)]; obj.pCAD.ObjImag(1).YData = [obj.pPos.Xc(2) obj.pPos.X(2)]; obj.pCAD.ObjImag(1).ZData = [0.1 0.1];
end

end

% =========================================================================
function mCADmake(obj)

obj.pCAD.i3D{1} = patch('Vertices',obj.pCAD.obj{1}.v','Faces',obj.pCAD.obj{1}.f3');

for ii = 1:length(obj.pCAD.obj{1}.umat3)
    mtlnum = obj.pCAD.obj{1}.umat3(ii);
    for jj=1:length(obj.pCAD.mtl{1})
        if strcmp(obj.pCAD.mtl{1}(jj).name,obj.pCAD.obj{1}.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = obj.pCAD.mtl{1}(jj).Kd';
    %fvcd(ii,:) = rand(1,3);
end

obj.pCAD.i3D{1}.FaceVertexCData = fvcd3;
obj.pCAD.i3D{1}.FaceColor = 'flat';
obj.pCAD.i3D{1}.EdgeColor = 'none';
obj.pCAD.i3D{1}.FaceAlpha = 0.0;
obj.pCAD.i3D{1}.Visible = 'off';
% light;

obj.pCAD.flagCreated = 1;

hold on
obj.pCAD.ObjImag(1) = plot3([obj.pPos.Xc(1) obj.pPos.X(1)],[obj.pPos.Xc(2) obj.pPos.X(2)],[0.1 0.1],'-*','Color',[1 1 1]*.5);

drawnow limitrate nocallbacks

end