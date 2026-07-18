classdef Cenario < handle
    %CENARIO Summary of this class goes here
    %   Detailed explanation goes here

    properties
        bebop
        obstaculos
        waypoints
        hTraj   % handle da linha de trajetória percorrida
    end

    methods
        function obj = Cenario(bebop, obstaculos, waypoints)
            obj.bebop = bebop;
            obj.obstaculos = obstaculos;
            obj.waypoints = waypoints;
        end
    end
    
    methods(Access=private)
        function renderizarObstaculos(obj)
        %RENDERIZAROBSTACULOS Renderizar obstáculos de acordo com suas
        %coordenadas e altura

            for i=1 : size(obj.obstaculos, 1)
                
                cx = obj.obstaculos(i, 1);  
                cy = obj.obstaculos(i, 2);
                sx = obj.obstaculos(i, 3);
                sy = obj.obstaculos(i, 4);
                h = obj.obstaculos(i, 5);

                x0 = cx-sx/2; % borda esquerda
                x1 = cx+sx/2; % borda direita
                y0 = cy-sy/2; % borda frontal
                y1 = cy+sy/2; % borda traseira

                v = [
                        x0 y0 0; 
                        x1 y0 0; 
                        x1 y1 0; 
                        x0 y1 0; 
                        x0 y0 h; 
                        x1 y0 h; 
                        x1 y1 h; 
                        x0 y1 h
                ];

                f = [
                        1 2 3 4; 
                        5 6 7 8; 
                        1 2 6 5; 
                        2 3 7 6; 
                        3 4 8 7; 
                        4 1 5 8
               ];
                

               % Renderiza o obstáculo
               patch('Vertices', v, 'Faces', f, 'FaceColor', [0.35 0.35 0.35], 'FaceAlpha', 0.55, 'EdgeColor', [0.2 0.2 0.2]);
                
               % Renderiza o nome do obstáculo
               nome = sprintf('Baia %s', char('A' + i - 1));
               text(cx, cy, h + 0.15, nome, 'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
            end
        end

        function renderizarWaypoints(obj)
        %RENDERIZARWAYPOINTS Renderiza os waypoints de acordo com suas
        %coordenadas. Baias (indBaia=1) em vermelho, vias (indBaia=0) em azul.

            if isempty(obj.waypoints)
                return;
            end

            for k = 1 : size(obj.waypoints, 1)
                x   = obj.waypoints(k, 1);
                y   = obj.waypoints(k, 2);
                z   = obj.waypoints(k, 3);
                tipo = obj.waypoints(k, 5);

                if tipo == 1
                    plot3(x, y, z, 'r.', 'MarkerSize', 22);
                else
                    plot3(x, y, z, 'b.', 'MarkerSize', 22);
                end
            end
        end

        function renderizarRotaPlanejada(obj)
        %RENDERIZARROTAPLANEJADA Renderiza a rota planejada com linha pontilhada
        
            if isempty(obj.waypoints)
                return;
            end

            xRota = [obj.bebop.pPos.X(1); obj.waypoints(:,1)];
            yRota = [obj.bebop.pPos.X(2); obj.waypoints(:,2)];
            zRota = [obj.bebop.pPos.X(3); obj.waypoints(:,3)];
            plot3(xRota, yRota, zRota, ':', 'Color', [.4 .4 .4], 'LineWidth', 1.5);
        end


    end

    methods
        function renderizar(obj)
        %RENDERIZAR Renderiza os obstáculos, o robô Bebop e seus
        %respectivos waypoints

            limites = [-7 7 -7 7 0 4];
            f1 = figure('Name','Inspecao em Estufa - Bebop 2','NumberTitle','off');
            f1.Position = [50 50 1200 850];    
            figure(f1);
            Ground = patch(limites([1 1 2 2]),limites([3 4 4 3]),[0 0 0 0],[0.6 1 0.6]);
            Ground.FaceAlpha = 0.3;
            view(-35,30), axis equal, axis(limites), grid on
            lighting phong; material shiny; lightangle(-45,30)
            
            obj.bebop.mCADplot;
            obj.bebop.mCADcolor([.80 .21 .21]);
            obj.bebop.mCADplot;
            
            hold on
            xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')

            obj.renderizarObstaculos()      % Renderizar obstáculos
            obj.renderizarWaypoints()       % Renderizar waypoints
            obj.renderizarRotaPlanejada()   % Renderizar rota planejada

            obj.hTraj = plot3(obj.bebop.pPos.X(1), obj.bebop.pPos.X(2), obj.bebop.pPos.X(3), ...
                              'b-', 'LineWidth', 1.5);

            rotate3d on                 % permite rotacionar a cena durante a simulação
            drawnow
        end

        function atualizar(obj)
        %ATUALIZAR Atualiza a posição do bebop e o rastro de trajetória

            obj.bebop.mCADplot;

            obj.hTraj.XData(end+1) = obj.bebop.pPos.X(1);
            obj.hTraj.YData(end+1) = obj.bebop.pPos.X(2);
            obj.hTraj.ZData(end+1) = obj.bebop.pPos.X(3);

            drawnow
        end
    end
end