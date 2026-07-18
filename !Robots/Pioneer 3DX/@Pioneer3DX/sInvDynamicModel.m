function sInvDynamicModel(p3dx)

% Determine the robot pose, based on the control signal
%  .  .     +-----------+       
% |X, U| -> |  Dynamic  |  ->  U_ref
%           |   Model   |      
%           +-----------+      
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


% - Modelo Dinâmica Inverso
D = [p3dx.pPar.theta(1) 0; 0 p3dx.pPar.theta(2)];
eta = [-p3dx.pPar.theta(3)*p3dx.pSC.u(2)^2 + p3dx.pPar.theta(4)*p3dx.pSC.u(1);
        p3dx.pPar.theta(5)*p3dx.pSC.u(1)*p3dx.pSC.u(2) + p3dx.pPar.theta(6)*p3dx.pSC.u(2)];

% - Velocidades de referência
p3dx.pSC.mu_inv = D*p3dx.pSC.du + eta;




