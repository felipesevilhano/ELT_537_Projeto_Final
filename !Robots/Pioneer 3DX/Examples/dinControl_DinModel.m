% Simulação em duas dimensões de um robô móvel diferencial

clc; clearvars; close all

% Looking for the root directory
FolderCurrent = pwd;

% Modify Root Folder if necessary.
FolderRoot = 'reference_scripts';
cd(FolderCurrent(1:(strfind(FolderCurrent,FolderRoot)+numel(FolderRoot)-1)))
addpath(genpath(pwd))

%% Características construtivas do robô
% Leader
p3dx(1) = Pioneer3DX;    % Pioneer (Robô 01) Leader
p3dx(1).sKinematicModel  % Atualiza a posição do ponto central do robô
p3dx(1).pPar.Ts = p3dx(1).pPar.Ts/10;

% - Parâmetros do modelo dinâmico do robô
p3dx(1).pPar.theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590]; % [Identified Parameters]
                                                                         % DOI: http://dx.doi.org/10.5772/intechopen.79397

% p3dx(1).pPar.delta = [0; 0.5; 0; 1.2; -0.8]; % parâmetros não modelados e incerteza paramétrica
p3dx(1).pPar.delta = [0; 0; 0; 0; 0];


%% Controle de Movimento
% - Ganhos controlador
k0 = 5; %700;
ki = k0^2/4;
Kpo = diag([k0 k0]);   % ganho do controle proporcional
Kpi = diag([ki ki]);   % ganho do controle derivativo

k0 = 5;
ki = k0^2/4;
Kdo = diag([k0 k0]);   % ganho do controle proporcional
Kdi = diag([ki ki]);   % ganho do controle derivativo

% - Controle cinemático
p3dx(1).pSC.mu_c = zeros(2,1);
p3dx(1).pSC.uc_ref = zeros(2,1);

% - Controle dinâmico
p3dx(1).pSC.mu = zeros(2,1);
p3dx(1).pSC.u_ref = zeros(2,1);

%% - Definição da tarefa a ser executada:
k_max = 2;       % iterador da tarefa (quantas vezes executará a tarefa)

%% Temporizadores para controle
T = 10;                         % período total do experimento 30s
T_amos = 0.02;                  % período da amostragem 0.02s ou 50 Hz
T_plot = .1;                    % período para cada atualização de plot 0.5s

dt = T_amos;                    % intervalo de tempo é igual ao período de amostragem 0.02s

%% - Armazenamento dos dados e plot de trajetórias e erros
% List of data to store (include others if necessary)
simDataVars = [p3dx(1).pPos.X' p3dx(1).pPos.Xd' p3dx(1).pPos.dX' p3dx(1).pPos.dXd'...
               p3dx(1).pSC.U' p3dx(1).pSC.Ud' ... 
               0];

% Prealocating memory to store the simulation data
T_amos = 0.02;                  % período da amostragem 0.02s ou 50 Hz
simData = zeros(ceil((T*k_max)/T_amos)+1,length(simDataVars));
simDataIdx = 1;

% Armazenando o estado inicial...
simData(simDataIdx,:) = simDataVars;
simDataIdx = simDataIdx + 1;

%% Plots
% - Criando a Figura que rodará a simulação (Ambiente de Simulação)
[Info,f] = initGraphics(T);
view(0,90)

% plots
l = .5; % comprimento da seta de orientação
xd = [p3dx(1).pPos.Xd(1); l*cos(p3dx(1).pPos.Xd(6))];
yd = [p3dx(1).pPos.Xd(2); l*sin(p3dx(1).pPos.Xd(6))];
ori_des = annotation('arrow','Linewidth',1,'Color','red');
ori_des.Parent = gca;           % associate the arrow the the current axes
ori_des.Position = [xd(1), yd(1), xd(2), yd(2)]; 

Xdes = plot(p3dx(1).pPos.Xd(1),p3dx(1).pPos.Xd(2),'xr','MarkerSize',15,'MarkerIndices',1);

h(1) = plot(p3dx(1).pPos.Xd(1),p3dx(1).pPos.Xd(2),'--k','LineWidth',.75);
h(2) = plot(p3dx(1).pPos.X(1),p3dx(1).pPos.X(2),'-ob','LineWidth',1,'MarkerIndices',1);
h(3) = plot([p3dx(1).pPos.X(1) p3dx(1).pPos.X(1)],[p3dx(1).pPos.X(2) p3dx(1).pPos.X(2)],'-*','Color',[.3 .3 .3],'LineWidth',1,'MarkerIndices',1);

lgd = legend([h(1) h(2)],{'$\textbf{h}_d$','$\textbf{h}$'},'FontSize',20,'interpreter','latex','Position',[0.82 0.52 0.084 0.10]);

idx = 2;

%% Loop de controle

for k = 1:k_max % repetição da tarefa
    if k == k_max
        ta = dt;
    else
        ta = 0;
    end

    for t = (k-1)*T:dt:k*T-T_amos+ta

        %% MALHA DE CONTROLE
        % Planejamento da tarefa no espaço operacional/cartesiano: x_EF_des, dx_EF_des e ddx_EF_des
        %p3dx(1) = planning(p3dx(1),'circle',t,T);
        p3dx(1) = planning(p3dx(1),'epitrocoid',t,T);

            
        %% Modelo dinâmico para o robô 
        % - matrizes de dinâmica
        f = [p3dx(1).pSC.U(1)*cos(p3dx(1).pPos.X(6)) - p3dx(1).pPar.a*p3dx(1).pSC.U(2)*sin(p3dx(1).pPos.X(6));
             p3dx(1).pSC.U(1)*sin(p3dx(1).pPos.X(6)) + p3dx(1).pPar.a*p3dx(1).pSC.U(2)*cos(p3dx(1).pPos.X(6));
             p3dx(1).pSC.U(2);
             p3dx(1).pPar.theta(3)/p3dx(1).pPar.theta(1)*p3dx(1).pSC.U(2)^2 - p3dx(1).pPar.theta(4)/p3dx(1).pPar.theta(1)*p3dx(1).pSC.U(1);
            -p3dx(1).pPar.theta(5)/p3dx(1).pPar.theta(2)*p3dx(1).pSC.U(1)*p3dx(1).pSC.U(2) - p3dx(1).pPar.theta(6)/p3dx(1).pPar.theta(2)*p3dx(1).pSC.U(2)];
        
        G = [0 0; 0 0; 0 0; 1/p3dx(1).pPar.theta(1) 0; 0 1/p3dx(1).pPar.theta(2)];
        p3dx(1).pPos.dX_dinamics = f + G*p3dx(1).pSC.u_ref;
        
        % Velocidade executada pelo robô
        p3dx(1).pSC.U = p3dx(1).pSC.U + p3dx(1).pPos.dX_dinamics(4:5)*dt;

                
        %% Modelo cinemático do robô
        % - Cinemática Direta
        % |  dx  |   | cos(pis)  -a sin(pis) | |  u  |
        % |  dy  | = | sin(psi)   a cos(pis) | |omega|
        % | dpsi |   |    0            1     | 
        Ac = [cos(p3dx(1).pPos.X(6)) -p3dx(1).pPar.a*sin(p3dx(1).pPos.X(6));
              sin(p3dx(1).pPos.X(6))  p3dx(1).pPar.a*cos(p3dx(1).pPos.X(6));
                      0                                1                  ];
        A = Ac(1:2,1:2); % Cinemática das coordenadas do ponto de interesse
        p3dx(1).pPos.dX([1 2 6]) = Ac*p3dx(1).pSC.U;


        % Robô (atua como integrador duplo u = ddq)
        p3dx(1).pPos.X([1 2 6]) = p3dx(1).pPos.X([1 2 6]) + p3dx(1).pPos.dX([1 2 6])*dt;  % pos_atual = pos_anterior + vel_atual*dt; Integração por Euler
        
        if abs(p3dx(1).pPos.X(6)) >= pi
            if p3dx(1).pPos.X(6) < 0
               p3dx(1).pPos.X(6) = p3dx(1).pPos.X(6) + 2*pi;
            else
                p3dx(1).pPos.X(6) = p3dx(1).pPos.X(6) - 2*pi;
            end
        end

        % Pose of the robot's center
        p3dx(1).pPos.Xc([1 2 6]) = p3dx(1).pPos.X([1 2 6]) - ...
            [cos(p3dx(1).pPos.X(6)) -sin(p3dx(1).pPos.X(6)) 0; sin(p3dx(1).pPos.X(6)) cos(p3dx(1).pPos.X(6)) 0; 0 0 1]*...
            [p3dx(1).pPar.a*cos(p3dx(1).pPar.alpha); p3dx(1).pPar.a*sin(p3dx(1).pPar.alpha); 0];


        %% Lei de controle (slide ...)
        p3dx(1).pPos.Xtil = p3dx(1).pPos.Xd - p3dx(1).pPos.X;
        p3dx(1).pPos.dXtil = p3dx(1).pPos.dXd - p3dx(1).pPos.dX;
        
        nu = p3dx(1).pPos.ddXd(1:2)' + Kdo*p3dx(1).pPos.dXtil(1:2) + Kpo*p3dx(1).pPos.Xtil(1:2);
%         nu = p3dx(1).pPos.ddXd(1:2) + Kdo*tanh(Kdo\Kdi*p3dx(1).pPos.dXtil(1:2)) + Kpo*tanh(Kpo\Kpi*p3dx(1).pPos.Xtil(1:2));

        % - controle de dinâmica inversa
        D = [p3dx(1).pPar.theta(1) 0; 0 p3dx(1).pPar.theta(2)];
        eta = [ p3dx(1).pPar.theta(3)/p3dx(1).pPar.theta(1)*p3dx(1).pSC.U(2)^2 - p3dx(1).pPar.theta(4)/p3dx(1).pPar.theta(1)*p3dx(1).pSC.U(1);
               -p3dx(1).pPar.theta(5)/p3dx(1).pPar.theta(2)*p3dx(1).pSC.U(1)*p3dx(1).pSC.U(2) - p3dx(1).pPar.theta(6)/p3dx(1).pPar.theta(2)*p3dx(1).pSC.U(2)];
        nu_a = [-p3dx(1).pSC.U(1)*p3dx(1).pSC.U(2)*sin(p3dx(1).pPos.X(6)) - p3dx(1).pPar.a*p3dx(1).pSC.U(2)^2*cos(p3dx(1).pPos.X(6));
                 p3dx(1).pSC.U(1)*p3dx(1).pSC.U(2)*cos(p3dx(1).pPos.X(6)) - p3dx(1).pPar.a*p3dx(1).pSC.U(2)^2*sin(p3dx(1).pPos.X(6))];
        Ainv = [cos(p3dx(1).pPos.X(6)) sin(p3dx(1).pPos.X(6));
               -1/p3dx(1).pPar.a*sin(p3dx(1).pPos.X(6)) 1/p3dx(1).pPar.a*cos(p3dx(1).pPos.X(6))];
        %p3dx(1).pSC.mu = D*(A\(nu - nu_a) + eta);
        p3dx(1).pSC.mu = D*(Ainv*(nu - nu_a) - eta);
        
        % Sinal de Controle desejado/referência
        p3dx(1).pSC.Ud = p3dx(1).pSC.mu;

        % Controle enviado ao robô
        p3dx(1).pSC.u_ref = p3dx(1).pSC.mu;   % controle acoplado

        % ---------------------------------------------------------------------
        %% FIM DA MALHA DE CONTROLE, AGORA ARMAZENAR DADOS E PLOT
        % - Storing simulation data
        % Variable to feed plotResults function
        simDataVars = [p3dx(1).pPos.X' p3dx(1).pPos.Xd' p3dx(1).pPos.dX' p3dx(1).pPos.dXd'...
                       p3dx(1).pSC.U' p3dx(1).pSC.Ud' ... 
                       t];

        simData(simDataIdx,:) = simDataVars;
        simDataIdx = simDataIdx + 1;

        % %    % 1 -- 12        13 -- 24      25 -- 36         37 -- 48
        % %    % P.pPos.X       P.pPos.Xd     P.pPos.X         P.pPos.dXd      
        % %    
        % %    % 49 -- 50       51 -- 52      
        % %    % P.pPos.U       P.pPos.Ud     
        % % 
        % %    % 53
        % %    toc(t)
        % ---------------------------------------------------------------------
    
        idx = idx - 1;
        if idx == 1
            idx = 5;

            time = t;             % Current Time
            Info.String = ['\textbf{Time}: ' num2str(time,'%05.2f') ' | ' num2str(k_max*T,'%05.2f') ' [s]~~~($1\times$)'];

            %% Desenha o robô
            % - desired position
            Xdes.XData = p3dx(1).pPos.Xd(1); Xdes.YData = p3dx(1).pPos.Xd(2);
            if t>0
                h(1).XData = [h(1).XData p3dx(1).pPos.Xd(1)]; h(1).YData = [h(1).YData p3dx(1).pPos.Xd(2)];
            else
                h(1).XData = [p3dx(1).pPos.Xd(1)]; h(1).YData = [p3dx(1).pPos.Xd(2)];
            end

            % - orientation arrow
            xd = [p3dx(1).pPos.Xd(1); l*cos(p3dx(1).pPos.Xd(6))];
            yd = [p3dx(1).pPos.Xd(2); l*sin(p3dx(1).pPos.Xd(6))];
            ori_des.Position = [xd(1), yd(1), xd(2), yd(2)];

            % - Robot Position
            h(2).XData = [h(2).XData p3dx(1).pPos.X(1)]; h(2).YData = [h(2).YData p3dx(1).pPos.X(2)];
            h(2).MarkerIndices = length(h(2).XData);

            % - Centro das rodas do robô
            h(3).XData = [p3dx(1).pPos.Xc(1) p3dx(1).pPos.X(1)]; h(3).YData = [p3dx(1).pPos.Xc(2) p3dx(1).pPos.X(2)];

            drawnow limitrate nocallbacks
        end
    end
end


%% - Finishing the experiments

% Close Robot Connection and Clear Variable
disp("Simulation ended succesfully...");

% Removing data not filled
simData(simDataIdx:end,:) = [];

% simulacion

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

%% Sinais de Controle (Fazer um lado do eixo y com velocidade linear e o outro com angular)
FigSC(1) = figure();
ax = gca;
ax.FontSize = 12;
yyaxis left
u(1:2) = plot(simData(:,end),simData(:,49),'-k',simData(:,end),simData(:,51),'--k','LineWidth',.75);
ax = gca;
ax.FontSize = 12;
ylabel({'$\tilde{u}$ [m/s]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
axis([0 simData(end,end) -1 4])

yyaxis right 
u(3:4) = plot(simData(:,end),simData(:,50),'-b',simData(:,end),simData(:,52),'--b','LineWidth',.75);
ylabel({'$\tilde{\omega}$ [rad/s]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
xlabel({'$t$ [s]'},'FontSize',14,'FontWeight','bold','interpreter','latex');

legend([u(:)],{'$u$','$u_d$','$\omega$','$\omega_d$'},'FontSize',16,'interpreter','latex','location','best','NumColumns',2)

hAx = gca;                       
set(hAx.YAxis,{'Color'},{'k'})   
grid on
axis([0 simData(end,end) -1 4])


%% Erros de posição:
figure();
subplot(211);
plot(simData(2:end,end),(simData(2:end,13)-simData(2:end,1)),'--k',simData(2:end,end),(simData(2:end,14)-simData(2:end,2)),'-.k','LineWidth',.75);
ax = gca;
ax.FontSize = 12;
xlabel({'$t$ [s]'},'FontSize',14,'FontWeight','bold','interpreter','latex');
ylabel({'$\tilde{\mathbf{x}}$ [m]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
legend({'$\tilde{x}$','$\tilde{y}$'},'FontSize',16,'interpreter','latex')%,'Position',[0.83 0.87 0.091 0.092])
grid on
axis tight
% Erro de Posi\c{c}{\~a}o

%% Erros de orientação:
subplot(212);
plot(simData(2:end,end),(simData(2:end,18)-simData(2:end,6)),'-k','LineWidth',.75);
ax = gca;
ax.FontSize = 12;
xlabel({'$t$ [s]'},'FontSize',14,'FontWeight','bold','interpreter','latex');
ylabel({'$\tilde{\mathbf{\psi}}$ [m]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
legend({'$\tilde{\psi}$'},'FontSize',16,'interpreter','latex')%,'box','off')
grid on
axis tight
