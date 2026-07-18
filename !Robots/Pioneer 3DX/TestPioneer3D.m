%%%% Pioneer P3-Dx: Rastreamento de Trajetória %%%%

clc, clear  
close all

% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz - Look for root directory
PastaAtual = pwd;
% Encontra o AuRoRa
beginAuRoRa = strfind(PastaAtual,'AuRoRA - EI NERo');

% Depois do \AuRoRA
endAuRoRaPath = strfind(PastaAtual(beginAuRoRa+6:end),'\');

% Se já não estiver no AuRoRa:
if ~isempty(endAuRoRaPath)
    % Encontra a pasta raiz do AuRoRa
    PastaRaiz = PastaAtual(1:beginAuRoRa+6+endAuRoRaPath(1)-1);
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
end


%% Classes initialization - Definindo o Robô
% Criando uma variável para representar o Robô
P = Pioneer3DX;
P.pPar.a = 0.0; % X Xc
P.pPar.alpha = 0*pi/3;

% Tempo de esperar para início do experimento/simulação
% clc;
fprintf('\nInício..............\n\n')
pause(1)

%% Definindo a Figura que irá rodar a simulação
% P.mPlotInit;
f1 = figure('Name','Simulação: Robótica Móvel (Pioneer P3-Dx)','NumberTitle','off');
% f1.Position = [435 2 930 682];
f1.Position = [1 2 930 682];
%     f1.Position = [1367 50 930 634]; % Quando uso segunda tela em Sete Lagoas!
figure(f1);

ax = gca;
ax.FontSize = 12;
xlabel({'$$x$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$y$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$z$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
view(3)
view(45,30)
grid on
hold on
grid minor
light;
axis([-1.5 1.5 -1.5 1.5 0 1.5])
set(gca,'Box','on');
% axis tight

%% P.mCADplot();
t = tic;
P.mCADplot;
drawnow

disp([num2str(toc(t)) 's para 1º plot'])


%% Teste para mudar a cor do Pioneer
% P(1) = Pioneer3DX;
% P(1).pPar.a = 0.1;
% P(2) = Pioneer3DX;
% P(2).pPar.a = 0.1;
% P(3) = Pioneer3DX;
% P(3).pPar.a = 0.1;
% P(4) = Pioneer3DX;
% P(4).pPar.a = 0.1;
% 
% Xo = [0 0 0 0];
% P.rSetPose(Xo);         % define pose do robô
% P(1).rSetPose(Xo + [0 1 0 0]);
% P(2).rSetPose(Xo + [0 -1 0 0]);
% P(3).rSetPose(Xo + [1 0 0 0]);
% P(4).rSetPose(Xo + [-1 0 0 0]);
% 
% P(1).mCADplot3D;
% P(2).mCADplot3D;
% P(3).mCADplot3D;
% P(4).mCADplot3D;
% 
% % Robot Appearance
% P.mCADcolor([0; 0.4470; 0.5410]);
% P(1).mCADcolor([0.4660 0.6740 0.1880]);
% P(2).mCADcolor([0.6350 0.0780 0.1840]);
% P(3).mCADcolor([0 0.4470 0.7410]);
% P(4).mCADcolor([0.9290 0.6940 0.1250]);
% 
% % P.pCAD.i3D.FaceAlpha = 0.5;
% 
% pause



%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
t = tic;
tc = tic;
tp = tic;

test = 0;


while toc(t) < 30
    
    if toc(tc) > .01
        
        tc = tic;
        
        if toc(t) > 40
            P.pPos.Xd(1:2) = [-.5; -.5];
        elseif toc(t) > 20
            P.pPos.Xd(1:2) = [.5; 0];
        else
            P.pPos.Xd(1:2) = [.5; .5];
        end
        
        % Data aquisition
        P.rGetSensorData;
        
        % Control
        
        % P = cKinematicController(P);
        % P = cDynamicController(P);
        P = cKinematicControllerExtended(P);

        
        % Send control to robot
        P.rSendControlSignals('kinematic');

    % Desenha o robô
    
%     if toc(tp) > tap

        tp = tic;

        P.mCADplot;
%         axis equal
%         grid on
        drawnow limitrate nocallbacks

%     end   
    
    end

end
%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals('kinematic');
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

pause(2)

axis equal
set(gca,'Box','on')
% close(v); % Fecha o vídeo

