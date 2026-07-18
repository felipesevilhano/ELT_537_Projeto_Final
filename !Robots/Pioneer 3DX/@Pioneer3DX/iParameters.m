function iParameters(p3dx)

p3dx.pPar.Model = 'P3-DX'; % robot model

% Sample time
p3dx.pPar.Ts = 0.1;  % For numerical integration
p3dx.pPar.ti = tic;  % Flag time
p3dx.pPar.t   = tic;  % Current Time

% Dynamic Model Parameters 
p3dx.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

% Robot mass [kg] 
p3dx.pPar.m = 0.429; %0.442;  

% Control Point "a" [m and rad] 
p3dx.pPar.a = 0.15;  % point of control
p3dx.pPar.alpha = 0; % angle of control

% Cinemática estendida do pioneer
p3dx.pPar.b = 0.25;

% [Identified Parameters]
% Reference: 
% Martins, F. N., & Brandão, A. S. (2018). 
% Motion Control and Velocity-Based Dynamic Compensation for Mobile Robots. 
% In Applications of Mobile Robots. IntechOpen.
% DOI: http://dx.doi.org/10.5772/intechopen.79397
p3dx.pPar.theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];

% Vertor de Incertezas Paramétricas
p3dx.pPar.delta = [0; 0; 0; 0; 0];

% Optitrack Pose filtering
p3dx.pPar.LKF.flag      = 0; % Inicialization
p3dx.pPar.LKF.xpOpt   = zeros(6,1); % Linear predictor
p3dx.pPar.LKF.mseOpt  = diag(randn(1,6)*0.01); % Mean square error
p3dx.pPar.LKF.varwOpt = diag([5e-7 5e-7 5e-7 5e-6 5e-6 5e-4]); % State/Observation variance
p3dx.pPar.LKF.varnOpt = diag([5e-7 5e-7 5e-7 8e-5 8e-5 8e-3]); % System variance

% Saturation values
p3dx.pPar.uSat    = zeros(6,1);
p3dx.pPar.uSat(1) = 1;  % Max roll  angle reference
p3dx.pPar.uSat(2) = 1;  % Max pitch angle reference
p3dx.pPar.uSat(3) = 1;  % Max yaw rate reference
p3dx.pPar.uSat(4) = 1;  % Max X Speed reference
p3dx.pPar.uSat(5) = 1;  % Max Y Speed reference
p3dx.pPar.uSat(6) = 1;  % Max Z Speed reference


