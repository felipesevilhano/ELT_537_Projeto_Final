function mCADdel(obj)
% função que apaga o robô 3D do cenário (em suma o deixa invisível)

    if isfield(obj.pCAD,'ObjImag')
        for idx = 1:length(obj.pCAD.i3D)
            obj.pCAD.i3D{idx}.Visible = 'off';
        end
    end

end