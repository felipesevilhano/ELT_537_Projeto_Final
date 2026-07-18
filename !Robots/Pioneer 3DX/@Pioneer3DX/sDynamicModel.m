function sDynamicModel(p3dx)

% Determine the robot pose, based on the control signal
%          +-----------+       .  .
% U_ref -> |  Dynamic  |  ->  |X, U|
%          |   Model   |      
%          +-----------+      
% 
% References:
% 
% Brandão, A. S. (2008). 
% Controle Descentralizado com Desvio de Obstáculos para uma Formação Líder-Seguidor de Robôs Móveis. 
% Dissertação de Mestrado. UFES.
% Página: 69 a 77
%
% Martins, F. N., & Brandão, A. S. (2018). 
% Motion Control and Velocity-Based Dynamic Compensation for Mobile Robots. 
% In Applications of Mobile Robots. IntechOpen.
% DOI: http://dx.doi.org/10.5772/intechopen.79397
%
% Martins, F. N., Sarcinelli-Filho, M., & Carelli, R. (2017). 
% A Velocity-Based Dynamic Model and Its Properties for Differential Drive Mobile Robots. 
% In Journal of Intell Robot Syst. Springer.
% DOI: http://dx.doi.org/10.1007/s10846-016-0381-9

p3dx.pSC.mu_ref = p3dx.pSC.Ud;
p3dx.pSC.u = p3dx.pSC.U;

% - matrizes de dinâmica
f = [p3dx.pSC.u(1)*cos(p3dx.pPos.X(3)) - p3dx.pPar.a*p3dx.pSC.u(2)*sin(p3dx.pPos.X(3));
     p3dx.pSC.u(1)*sin(p3dx.pPos.X(3)) + p3dx.pPar.a*p3dx.pSC.u(2)*cos(p3dx.pPos.X(3));
     p3dx.pSC.u(2);
     p3dx.pPar.theta(3)/p3dx.pPar.theta(1)*p3dx.pSC.u(2)^2 - p3dx.pPar.theta(4)/p3dx.pPar.theta(1)*p3dx.pSC.u(1);
    -p3dx.pPar.theta(5)/p3dx.pPar.theta(2)*p3dx.pSC.u(1)*p3dx.pSC.u(2) - p3dx.pPar.theta(6)/p3dx.pPar.theta(2)*p3dx.pSC.u(2)];

G = [0 0; 0 0; 0 0; 1/p3dx.pPar.theta(1) 0; 0 p3dx.pPar.theta(2)];


% first-time derivative of the current position
p3dx.pPos.dX_dynamics = f + G*p3dx.pSC.mu_ref + p3dx.pPar.delta;

p3dx.pSC.du = p3dx.pPos.dX_dynamics(4:5);

% Velocidades linear e angular do robô (u e omega)
p3dx.pSC.U = p3dx.pSC.U + p3dx.pSC.du*p3dx.pPar.Ts;

% -------------------------------------------------------------------------
% - Cinemática Direta
% |  dx  |   | cos(pis)  -a sin(pis) | |  u  |
% |  dy  | = | sin(psi)   a cos(pis) | |omega|
% | dpsi |   |    0            1     |
K = [cos(p3dx.pPos.X(6)) -p3dx.pPar.a*sin(p3dx.pPos.X(6)+p3dx.pPar.alpha); ...
     sin(p3dx.pPos.X(6))  p3dx.pPar.a*cos(p3dx.pPos.X(6)+p3dx.pPar.alpha); ...
             0                             1                   ];

% first-time derivative of the current position
p3dx.pPos.X([7 8 12]) = K*p3dx.pSC.U(1:2);

% Robô (atua como integrador duplo u = ddq)
p3dx.pPos.X([1 2 6]) = p3dx.pPos.X([1 2 6]) + p3dx.pPos.X([7 8 12])*p3dx.pPar.Ts;  % pos_atual = pos_anterior + vel_atual*dt; Integração por Euler

% Angle limitation per quadrant
for ii = 4:6
    if abs(p3dx.pPos.X(ii)) > pi
        if p3dx.pPos.X(ii) < 0
            p3dx.pPos.X(ii) = p3dx.pPos.X(ii) + 2*pi;
        else
            p3dx.pPos.X(ii) = p3dx.pPos.X(ii) - 2*pi;
        end
    end
end

% -------------------------------------------------------------------------
% Pose of the robot's center
p3dx.pPos.Xc([1 2 6]) = p3dx.pPos.X([1 2 6]) - ...
    [cos(p3dx.pPos.X(6)) -sin(p3dx.pPos.X(6)) 0; sin(p3dx.pPos.X(6)) cos(p3dx.pPos.X(6)) 0; 0 0 1]*...
    [p3dx.pPar.a*cos(p3dx.pPar.alpha); p3dx.pPar.a*sin(p3dx.pPar.alpha); 0];


% -------------------------------------------------------------------------

% - Modelo Dinâmica Inverso
D = [p3dx.pPar.theta(1) 0; 0 p3dx.pPar.theta(2)];
eta = [-p3dx.pPar.theta(3)*p3dx.pSC.u(2)^2 + p3dx.pPar.theta(4)*p3dx.pSC.u(1);
        p3dx.pPar.theta(5)*p3dx.pSC.u(1)*p3dx.pSC.u(2) + p3dx.pPar.theta(6)*p3dx.pSC.u(2)];
p3dx.pSC.mu_inv = D*p3dx.pSC.du + eta;




