classdef Simulacao < handle
    %SIMULACAO Classe para simulação util
    %   Permite simular o projeto final da disciplina ELT 537

    properties(Constant)
        BLOCO_DADOS_SIMULACAO = 1000;   % Tamanho de cada bloco de armazenamento de dados
        NPASSO_PLOT = 2;                % Quantidade mínima de passos necessários para renderizar a posição do bebop
        T_CONTROLE = 1/30;              % Amostragem do bebop
        
        % parâmetros do modelo (Bebop 2)
        K1 = 0.8417;  K2 = 0.18227;   % Eixo x
        K3 = 0.8354;  K4 = 0.17095;   % Eixo y
        K5 = 3.966;   K6 = 4.001;     % Eixo z
        K7 = 9.8524;  K8 = 4.7295;    % Eixo psi

        F1 = diag([0.8417  0.8354  3.966   9.8524]);   % IMPARES  -> Atuam no sinal de controle  [K1 K3 K5 K7]
        F2 = diag([0.18227 0.17095 4.001   4.7295]);   % PARES    -> Atuam no arrasto/velocidade [K2 K4 K6 K8]

        % Ganhos do modelo
        Kp = diag([1.0  1.0  1.5  1.0]);   % Ganho proporcional
        Kd = diag([1.0  1.0  1.5  1.0]);   % Ganho derivativo

        % Tolerâncias
        TOL_BAIA = 0.10;        % Tolerância para waypoints de baia
        TOL_VIA = 0.30;         % Tolerância para waypoints de via
        TOL_ORI = deg2rad(2);   % Tolerância para orientações (Utilizado em waypoints de baia)
    end

    properties
        bebop
        cenario
        obstaculos
        waypoints
        dadosSimulacao
        simulacaoCompleta
    end
    
    methods
        function obj = Simulacao()
        %SIMULACAO Construtor da classe Simulacao
        %
        %   Retorno:
        %       OBJ - Objeto da classe Simulacao inicializado

            % Obstáculos no padrão [x_centro, y_centro, largura, profundidade, altura]
            obj.obstaculos = [
                -3.25  -2.25  2.0  3.5  2.0; % Baia A
                -3.25    3.0  2.0  3.5  2.0; % Baia B
                 2.75    3.0  2.0  3.5  2.0; % Baia C
                 2.75  -2.25  2.0  3.5  2.0; % Baia D
            ];

            % Pontos alvo no padrão [x, y, z, angulo_orientacao, indBaia]
            obj.waypoints = [           
                0.0  -5.0  1.0   90  0; % Decolagem
                0.0   0.5  3.0    0  0; % Via → Baia A (Manobra)
               -3.0   0.0  3.0  -90  1; % Baia A
               -6.5   0.5  3.0    0  0; % Via → Baia B (Manobra)
               -5.0   3.0  2.0    0  1; % Baia B
               -5.0   6.0  3.0    0  0; % Via → Baia C (Manobra)
                7.0   6.0  3.0    0  0; % Via → Baia C (Manobra)
                5.0   2.0  1.0  180  1; % Baia C
                5.5  -5.5  3.0    0  0; % Via → Baia D (Manobra)
                3.0  -5.0  3.0   90  1; % Baia D
                0.0  -5.0  1.0   90  1; % Base
            ];

            obj.waypoints(:, 4) = deg2rad(obj.waypoints(:, 4));   % converte psi para radianos

            obj.bebop = Simulacao.inicializarBebop();                           % Inicializa o Bebop
            obj.cenario = Cenario(obj.bebop, obj.obstaculos, obj.waypoints);    % Inicializa o cenário
            obj.dadosSimulacao = zeros(0, 29);                                  % Reserva espaço para armazenamento dos dados da simulação
            obj.simulacaoCompleta = false;                                      % Indicada se a simulação terminou
        end

        function simular(obj)
        %SIMULAR Executa a simulação
        %
        %   Descrição:
        %       Executa a simulação do bebop em tempo real
           
           
            obj.cenario.renderizar();   % Renderiza o cenário
            
            t_controle = tic;   % Tempo de controle de amostragem
            t_simulacao = 0;    % Tempo de simulação
            nPasso = 0;         % Contador de passos executados
            waypointAtual = 1;  % Indicador do waypoint atual
            
            while ~obj.simulacaoCompleta

                % Trava para executar apenas um passo por amostragem
                if toc(t_controle) < obj.T_CONTROLE
                    continue;
                end
                
                t_controle = tic;
                t_simulacao = t_simulacao + obj.T_CONTROLE;
                nPasso = nPasso + 1;
   
                % Indicador de decolagem
                if strcmp(obj.bebop.pPar.fightState, 'landed') && t_simulacao > 0.5
                    obj.bebop.pPar.fightState = 'taking-off';
                end

                % Leitura dos dados de sensores
                obj.bebop.rGetSensorData;

                % Posição desejada
                obj.bebop.pPos.Xd(:) = 0;
                obj.bebop.pPos.Xd([1 2 3 6]) = obj.waypoints(waypointAtual, 1:4);   % Posição desejada (x, y, z, psi)
                
                % Cálculo do erro
                obj.bebop.pPos.Xtil = obj.bebop.pPos.Xd - obj.bebop.pPos.X;                                 % Posição desejada - Posição atual
                obj.bebop.pPos.Xtil(6) = atan2(sin(obj.bebop.pPos.Xtil(6)),cos(obj.bebop.pPos.Xtil(6)));    % Wrap do erro psi

                % Controlador
                psi = obj.bebop.pPos.X(6);         

                R = [
                    cos(psi)  -sin(psi)  0  0 ;
                    sin(psi)   cos(psi)  0  0 ;
                           0          0  1  0 ;
                           0          0  0  1
                ];
            
                Xtil  = obj.bebop.pPos.Xtil([1 2 3 6]);    % Erro de posicao   (x ,y, z, psi)
                dXtil = obj.bebop.pPos.Xtil([7 8 9 12]);   % Erro de velocidade
                dX    = obj.bebop.pPos.X([7 8 9 12]);      % Velocidade atual
            
                eta = obj.Kd*dXtil + obj.Kp*tanh(Xtil);             
                U   = (R*obj.F1) \ ( eta + R*obj.F2*dX );  % Lei de controle
                U = min(max(U,-1),1);      

                % Configura as velocidades desejadas de acordo com o que foi calculado pela lei de controle
                obj.bebop.pSC.Ud([1 2 3 6]) = U;           
           
                % Envia sinais para o bebop
                obj.bebop.rSendControlSignals;

                % Analisa próximo waypoint
                rho = norm(obj.bebop.pPos.Xtil(1:3));   % Erro de posição

                if obj.waypoints(waypointAtual, 5) == 1 % Verifica se é baia
                    % Caso seja baia, considera o erro de posição e orientação
                    waypointConcluido = (rho < obj.TOL_BAIA) && (abs(obj.bebop.pPos.Xtil(6)) < obj.TOL_ORI);
                else
                    % Caso seja via, considera o erro de posição
                    waypointConcluido = (rho < obj.TOL_VIA);
                end

                if waypointConcluido
                    if waypointAtual < size(obj.waypoints, 1)
                        waypointAtual = waypointAtual + 1;
                    else
                        obj.simulacaoCompleta = true;
                    end
                end

                % Armazena os dados do Bebop
                obj.armazenarDadosSimulacao(nPasso, [obj.bebop.pPos.Xd'  obj.bebop.pPos.X'  U'  t_simulacao]);

                % Renderiza a posição do Bebop
                if mod(nPasso, obj.NPASSO_PLOT) == 0
                    obj.cenario.atualizar;
                end
            end
            
            % Descarta linhas não preenchidas
            obj.dadosSimulacao = obj.dadosSimulacao(1:nPasso, :);

            % Plotar gráficos
            obj.plotarResultados();
        end

        function plotarResultados(obj)
        %PLOTARRESULTADOS Análise gráfica pós-simulação
        %   dadosSimulacao: cols 1:12=Xd | 13:24=X | 25:28=U | 29=tempo

            d = obj.dadosSimulacao;
            t = d(:, 29);

            % Índices
            iX   = 13;  iXd  = 1;
            iY   = 14;  iYd  = 2;
            iZ   = 15;  iZd  = 3;
            iPsi = 18;  iPsid = 6;

            % Gráfico: Navegação XYZ
            figure('Name','Navegação XYZ')
            plot3(d(:,iX), d(:,iY), d(:,iZ), 'b-', 'LineWidth', 1.5)
            xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
            title('Navegação em XYZ'), grid on, view(-35, 30)

            % Gráfico: Evolução temporal de X
            figure('Name','Evolução de X')
            plot(t, d(:,iXd), 'r--', t, d(:,iX), 'b-', 'LineWidth', 1.5)
            xlabel('t [s]'), ylabel('x [m]')
            title('Evolução temporal de X')
            legend('x_d', 'x'), grid on

            % Gráfico: Evolução temporal de Y 
            figure('Name','Evolução de Y')
            plot(t, d(:,iYd), 'r--', t, d(:,iY), 'b-', 'LineWidth', 1.5)
            xlabel('t [s]'), ylabel('y [m]')
            title('Evolução temporal de Y')
            legend('y_d', 'y'), grid on

            % Gráfico: Evolução temporal de Z
            figure('Name','Evolução de Z')
            plot(t, d(:,iZd), 'r--', t, d(:,iZ), 'b-', 'LineWidth', 1.5)
            xlabel('t [s]'), ylabel('z [m]')
            title('Evolução temporal de Z')
            legend('z_d', 'z'), grid on

            % Gráfico: Evolução temporal de psi
            figure('Name','Evolução de \psi')
            plot(t, rad2deg(d(:,iPsid)), 'r--', t, rad2deg(d(:,iPsi)), 'b-', 'LineWidth', 1.5)
            xlabel('t [s]'), ylabel('\psi [graus]')
            title('Evolução temporal de \psi (yaw)')
            legend('\psi_d', '\psi'), grid on

            % ── Sinais de controle ────────────────────────────────────────
            figure('Name','Sinais de controle')
            nomes = {'u_x','u_y','u_z','u_\psi'};
            for k = 1:4
                subplot(4,1,k)
                plot(t, d(:, 24+k), 'b-', 'LineWidth', 1.2)
                ylabel(nomes{k}), ylim([-1.1 1.1]), grid on
                if k == 1, title('Sinais de controle U'), end
                if k == 4, xlabel('t [s]'), end
            end
        end
    end

    methods(Access = private)
        function armazenarDadosSimulacao(obj, nPasso, dados)
            if nPasso > size(obj.dadosSimulacao, 1)
                obj.dadosSimulacao(end + obj.BLOCO_DADOS_SIMULACAO, :) = 0;
            end
            obj.dadosSimulacao(nPasso, :) = dados;
        end
    end

    methods(Static, Access = private)
        function bebop = inicializarBebop()
        %INICIALIZARBEBOP Criar e inicializar o Bebop
        %
        % Retorno
        %   B : Bebop
        %       Instância configurada e pronta para uso
        %
        % Descrição
        %   Inicializa a posição do drone, define a origem, coloca o estado
        %   de voo como "landed" e realiza uma leitura inicial dos sensores

            B = Bebop(1);
            B.pPos.X(1) = 0;                % Coordenada atual em X
            B.pPos.X(2) = -5;               % Coordenada atual em Y
            B.pPos.X(3) = 0;                % Coordenada atual em Z
            B.pPos.Xo(1:3) = B.pPos.X(1:3); % Coordenada de origem
            B.pPar.fightState = 'landed';   % Estado inicial 'pousado'
            B.rGetSensorData;               % Leitura dos dados do sensor

            bebop = B;
        end
    end
end