function mCADcolor(drone,color)
% Modify drone color

if nargin > 1
    drone.pCAD.mtl{1}(2).Kd = color'; % 3 -> no outro arquivo, mais pesado
    drone.pCAD.i3D{2}.FaceColor = color';
    drone.pCAD.i3D{4}.FaceColor = color';

end


for ii = 1:length(drone.pCAD.obj{1}.umat3)
    mtlnum = drone.pCAD.obj{1}.umat3(ii);
    for jj=1:length(drone.pCAD.mtl{1})
        if strcmp(drone.pCAD.mtl{1}(jj).name,drone.pCAD.obj{1}.usemtl(mtlnum-1))
            break;
        end
    end
    fvcd3(ii,:) = drone.pCAD.mtl{1}(jj).Kd';
end

drone.pCAD.i3D{1}.FaceVertexCData  = fvcd3;


end