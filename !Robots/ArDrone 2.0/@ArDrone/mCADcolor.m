function mCADcolor(drone,color)
% Modify drone color

if nargin > 1
    
    % Drone Indoor
    drone.pCAD.mtl{1}(4).Kd = color';
    
    % Outdoor
    drone.pCAD.mtl{2}(3).Kd = color';
    drone.pCAD.mtl{3}(2).Kd = color';

end


for i = 1:2
    
    fvcd3 = [];
    
    for ii = 1:length(drone.pCAD.obj{i}.umat3)
        mtlnum = drone.pCAD.obj{i}.umat3(ii);
        for jj=1:length(drone.pCAD.mtl{i})
            if strcmp(drone.pCAD.mtl{i}(jj).name,drone.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = drone.pCAD.mtl{i}(jj).Kd';
    end

    drone.pCAD.i3D{i}.FaceVertexCData  = fvcd3;
end


% --
a = 3;
    
fvcd3 = [];

for ii = 1:length(drone.pCAD.obj{a}.umat3)
    mtlnum = drone.pCAD.obj{a}.umat3(ii);
    for jj=1:length(drone.pCAD.mtl{a})
        if strcmp(drone.pCAD.mtl{a}(jj).name,drone.pCAD.obj{a}.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = drone.pCAD.mtl{a}(jj).Kd';
end

drone.pCAD.i3D{3}.FaceVertexCData  = fvcd3;
drone.pCAD.i3D{5}.FaceVertexCData  = fvcd3;
  

end

