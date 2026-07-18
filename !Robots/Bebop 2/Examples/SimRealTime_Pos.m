% Testar modelo gráfico em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end

% - Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% - Representação do robô
A = Bebop;

% Alterando posição
A.pPos.X(3) = 0.75;


tmax = 60; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

limites = [-1 1 -1 1 0 1];
figure(1)
% Ground = patch(limites([1 1 2 2]),limites([3 4 4 3]),limites([5 5 5 5]),[0.6 1 0.6]);
% Ground.FaceAlpha = 0.4;
% view(40,40)
view(45,30)

axis equal
axis(limites)
grid on

% Estilizando superficie
lighting phong;
material shiny;
colormap winter;
lightangle(-45,30)


Info = title(['Time: ' num2str(0,'%05.2f') ' | ' num2str(tmax,'%05.2f') 's']);  

A.mCADplot();

% Alterando cor do robô
A.mCADcolor([0 51 80]/255);

drawnow
pause(2)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];

% - como ainda não temos modelo dinâmico ou controlador...
stepTime = 1/30;
n = numel(0:stepTime:tmax);

x = linspace(A.pPos.X(1),0.5,n/2);
y = linspace(A.pPos.X(2),0.75,n/2); 
z = linspace(A.pPos.X(3),0.65,n/2); 
phi = linspace(A.pPos.X(4),0,n/2);
theta = linspace(A.pPos.X(5),0,n/2); 
psi = linspace(A.pPos.X(6),-pi/2,n/2);

idx = 1;


while toc(t) < tmax
    ta = tic;
    if toc(tc) > 1/30/2
        TT = [TT toc(tc)];
        
        
        tc = tic;
        
        if idx < length(x)        
            A.pPos.X(1:6) = [x(idx) y(idx) z(idx) phi(idx) theta(idx) psi(idx)];
        end
        
        idx = idx + 1;
    
    end
    if toc(tp) > 0.1
        tp = tic;
        A.mCADplot;
        
        Info.String = ['Time: ' num2str(toc(t),'%05.2f') ' | ' num2str(tmax,'%05.2f') 's'];
        
        drawnow
    end
    toc(ta)
end

% figure
% subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
% legend('\phi_{Des}','\phi_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
% legend('\theta_{Des}','\theta_{Atu}')
% grid
% 
% figure
% plot(XX([1,13],:)',XX([2,14],:)')
% axis equal
% 
% figure
% subplot(211),plot(XX(end,:),XX([1 13],:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([2 14],:)')
% legend('y_{Des}','y_{Atu}')
% grid

% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

