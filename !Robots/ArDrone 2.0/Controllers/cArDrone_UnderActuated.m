function [drone,cGains,cParameters] = cArDrone_UnderActuated(drone,cGains,cParameters)

% [drone,cGains,cParameters] = cArDrone_UnderActuated(drone,cGains,cParameters)
%
% Dynamic controller based on its dynamic model
% Brandão, A. S., M. Sarcinelli-Filho, and R. Carelli.
% "High-level underactuated nonlinear control for rotorcraft machines."
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% ArDrone 2.0 Parameters
% Li, Qianying. "Grey-box system identification of a quadrotor unmanned
% aerial vehicle." Master of Science Thesis Delft University of
% Technology (2014).
%
% Controllers Gains.
% The Gains must be given in the folowing order
% [kxL1  kyL1 kzL1 kPhiL1 kthetaL1 kPsiL1 ; kxL2 kyL2  kzL2 kPhiL2 kthetaL2 kPsiL2]
% cGains = [0.5 0.5 5 1 1 1; 2 2 2 20 20 2.5];


if nargin < 3
    % List of controller paramenters
    % ArDrone 2.0 Paramenters

    cParameters.Ts  = 1/30; % Time sample

    % Dynamic Model Parameters
    cParameters.g = 9.8;    % [kg.m/s^2] Gravitational acceleration
    cParameters.m = 0.429; %0.442;  % [kg] ArDrone mass

    % [kg.m^2] Moments of Inertia
    cParameters.Ixx = 2.237568e-3; % 9.57*1e-3;
    cParameters.Iyy = 2.985236e-3; % 9.57*1e-3;
    cParameters.Izz = 4.80374e-3;  % 25.55*1e-3;

    cParameters.Ixy = 0;
    cParameters.Ixz = 0;
    cParameters.Iyz = 0;

    % Rotor Parameters
    cParameters.r = 8.625; % Reduction Gear
    cParameters.R = 0.6029; % Motor resistance
    cParameters.Jm = 0.1215; %2.029585e-5;
    cParameters.Bm = 3.7400; %1.06e-3;
    cParameters.Km = 1.3014e2; %0.39;
    cParameters.Kb = 1.3014e-3; %8e-5;

    cParameters.Cf = 8.048e-6;
    cParameters.Ct = 2.423e-7;

    % Low-level PD controller gains
    cParameters.kdp = 0.5;
    cParameters.kpp = 0.1;
    cParameters.kdt = 0.5;
    cParameters.kpt = 0.10;
    cParameters.kds = 0.05;
    cParameters.kps = 0.1;
    cParameters.kdz = 0.01;
    cParameters.kpz = 0.1;

    % Propeller coeficients
    cParameters.k1 = 0.1785;
    cParameters.k2 = cParameters.Ct/cParameters.Cf;

    % Saturation values
    %     pitch          | [-1,1] <==> [-15,15] degrees
    %     roll           | [-1,1] <==> [-15,15] degrees
    %     altitude rate  | [-1,1] <==> [-1,1] m/s
    %     yaw rate       | [-1,1] <==> [-100,100] degrees/s
    cParameters.uSat    = zeros(4,1);
    cParameters.uSat(1) = 30*pi/180;  % Max roll  angle reference
    cParameters.uSat(2) = 30*pi/180;  % Max pitch angle reference
    cParameters.uSat(3) = 1;          % Max altitude rate reference
    cParameters.uSat(4) = 100*pi/180; % Max yaw rate reference

    % Motor voltage in hovering stage
    cParameters.Wo = sqrt(cParameters.m*cParameters.g/4/cParameters.Cf);

    cParameters.Vo = (cParameters.R*cParameters.Bm/cParameters.Km + cParameters.Kb)*cParameters.Wo + ...
        cParameters.R/cParameters.r/cParameters.Km*cParameters.Ct*cParameters.Wo^2;

    % Rotor velocities
    cParameters.W = zeros(4,1);

    % Model disturbances
    cParameters.D = zeros(6,1);
    cParameters.Q = zeros(3,1);
end

if nargin < 2
    % Default Controller Gains
    cGains = [0.5 0.5 2 5 5 1; 2.5 2.5 10 15 15 2.5];
    % disp('Gains not given. Using standard ones.');
end

% -------------------------------------------------------------------------
Ganhos.kx1 = cGains(1,1);
Ganhos.kx2 = cGains(2,1);
Ganhos.kx3 = sqrt(4*Ganhos.kx1);
Ganhos.kx4 = sqrt(4*Ganhos.kx1*Ganhos.kx2)/Ganhos.kx3;

Ganhos.ky1 = cGains(1,2);
Ganhos.ky2 = cGains(2,2);
Ganhos.ky3 = sqrt(4*Ganhos.ky1);
Ganhos.ky4 = sqrt(4*Ganhos.ky1*Ganhos.ky2)/Ganhos.ky3;

Ganhos.kz1 = cGains(1,3);
Ganhos.kz2 = cGains(2,3);
Ganhos.kz3 = sqrt(4*Ganhos.kz1);
Ganhos.kz4 = sqrt(4*Ganhos.kz1*Ganhos.kz2)/Ganhos.kz3;

% phi
Ganhos.kp1 = cGains(1,4);
Ganhos.kp2 = cGains(2,4);
Ganhos.kp3 = sqrt(4*Ganhos.kp1);
Ganhos.kp4 = sqrt(4*Ganhos.kp1*Ganhos.kp2)/Ganhos.kp3;
% theta
Ganhos.kt1 = cGains(1,5);
Ganhos.kt2 = cGains(2,5);
Ganhos.kt3 = sqrt(4*Ganhos.kt1);
Ganhos.kt4 = sqrt(4*Ganhos.kt1*Ganhos.kt2)/Ganhos.kt3;
%psi
Ganhos.ks1 = cGains(1,6);
Ganhos.ks2 = cGains(2,6);
Ganhos.ks3 = sqrt(4*Ganhos.ks1);
Ganhos.ks4 = sqrt(4*Ganhos.ks1*Ganhos.ks2)/Ganhos.ks3;

% % ---------------------------------------------------------
drone.pPos.Xda = drone.pPos.Xd;

% Calculando erro de posição
drone.pPos.Xtil = drone.pPos.Xd - drone.pPos.X;

% Matriz de rotação
Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];

R = (Rz*Ry*Rx);

%-------------------------------
% Controle Cinematico
%-------------------------------
% Position
etax = drone.pPos.dXd(7) + Ganhos.kx1*tanh(Ganhos.kx2*drone.pPos.Xtil(1)) + Ganhos.kx3*tanh(Ganhos.kx4*drone.pPos.Xtil(7));
etay = drone.pPos.dXd(8) + Ganhos.ky1*tanh(Ganhos.ky2*drone.pPos.Xtil(2)) + Ganhos.ky3*tanh(Ganhos.ky4*drone.pPos.Xtil(8));
etaz = drone.pPos.dXd(9) + Ganhos.kz1*tanh(Ganhos.kz2*drone.pPos.Xtil(3)) + Ganhos.kz3*tanh(Ganhos.kz4*drone.pPos.Xtil(9));


% Referência de Rolagem e Arfagem
drone.pPos.Xd(4) =  atan2((etax*sin(drone.pPos.X(6))-etay*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(etaz+cParameters.g));
drone.pPos.Xd(5) =  atan2((etax*cos(drone.pPos.X(6))+etay*sin(drone.pPos.X(6))),(etaz+cParameters.g));

% (Inserir Filtragem)
% drone.pPos.Xd(10) = (drone.pPos.Xd(4)-drone.pPos.Xda(4))/cParameters.Ts;
% drone.pPos.Xd(11) = (drone.pPos.Xd(5)-drone.pPos.Xda(5))/cParameters.Ts;

% Calculando erro de posição
drone.pPos.Xtil = drone.pPos.Xd - drone.pPos.X;

% Correction yaw error
if abs(drone.pPos.Xtil(6)) > pi
    if drone.pPos.Xtil(6) > 0
        drone.pPos.Xtil(6) = -2*pi + drone.pPos.Xtil(6);
    else
        drone.pPos.Xtil(6) =  2*pi + drone.pPos.Xtil(6);
    end
end


% Attitude
etap = drone.pPos.dXd(10) + Ganhos.kp1*tanh(Ganhos.kp2*drone.pPos.Xtil(4)) + Ganhos.kp3*tanh(Ganhos.kp4*drone.pPos.Xtil(10));
etat = drone.pPos.dXd(11) + Ganhos.kt1*tanh(Ganhos.kt2*drone.pPos.Xtil(5)) + Ganhos.kt3*tanh(Ganhos.kt4*drone.pPos.Xtil(11));
etas = drone.pPos.dXd(12) + Ganhos.ks1*tanh(Ganhos.ks2*drone.pPos.Xtil(6)) + Ganhos.ks3*tanh(Ganhos.ks4*drone.pPos.Xtil(12));

% =========================================================================
% Parte Translacional
Mt = cParameters.m*eye(3,3);              % Inertia matrix
Ct = zeros(3,3);                         % Coriolis matrix
Gt = [0; 0; cParameters.m*cParameters.g];  % Gravity matrix

% =========================================================================
% Rotational inertia matrix
Mr = [cParameters.Ixx, ...
    cParameters.Ixy*cos(drone.pPos.X(4)) - cParameters.Ixz*sin(drone.pPos.X(4)), ...
    -cParameters.Ixx*sin(drone.pPos.X(5)) + cParameters.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + cParameters.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5));

    cParameters.Ixy*cos(drone.pPos.X(4)) - cParameters.Ixz*sin(drone.pPos.X(4)), ...
    cParameters.Iyy*cos(drone.pPos.X(4))^2 + cParameters.Izz*sin(drone.pPos.X(4))^2 - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)),...
    cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - cParameters.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5));

    -cParameters.Ixx*sin(drone.pPos.X(5)) + cParameters.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + cParameters.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)), ...
    cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - cParameters.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5)),...
    cParameters.Ixx*sin(drone.pPos.X(5))^2 + cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - 2*cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - 2*cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2
    ];

% Rotational Coriolis matrix
Cr = [ 0, ...
    drone.pPos.X(11)*(cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + cParameters.Iyz*cos(drone.pPos.X(4))^2 - cParameters.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(-cParameters.Ixx*cos(drone.pPos.X(5))/2 - cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(11)*(-cParameters.Ixx*cos(drone.pPos.X(5))/2 - cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + cParameters.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2);

    drone.pPos.X(10)*(-cParameters.Ixy*sin(drone.pPos.X(4)) - cParameters.Ixz*cos(drone.pPos.X(4))) + drone.pPos.X(11)*(-cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - cParameters.Iyz*cos(drone.pPos.X(4))^2 + cParameters.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(cParameters.Ixx*cos(drone.pPos.X(5))/2 + cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(-cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - cParameters.Iyz*cos(drone.pPos.X(4))^2 + cParameters.Iyz*sin(drone.pPos.X(4))^2),...
    drone.pPos.X(10)*(cParameters.Ixx*cos(drone.pPos.X(5))/2 + cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-cParameters.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + cParameters.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)));

    drone.pPos.X(10)*(cParameters.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - cParameters.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-cParameters.Ixx*cos(drone.pPos.X(5))/2 + cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - cParameters.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2),...
    drone.pPos.X(10)*(-cParameters.Ixx*cos(drone.pPos.X(5))/2 + cParameters.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - cParameters.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + cParameters.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - cParameters.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) + cParameters.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + cParameters.Iyz*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5)) - cParameters.Iyz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))) + drone.pPos.X(12)*(cParameters.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - cParameters.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(cParameters.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - cParameters.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + cParameters.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - cParameters.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2) + drone.pPos.X(11)*(cParameters.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - cParameters.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - cParameters.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + cParameters.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*cParameters.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)))
    ];

% Gravity vector
Gr = [0; 0; 0];

% Modelo no formato: M \ddot{q} + C \dot{q} + G = F
Z = zeros(3,3);

MM = [Mt Z; Z Mr];   % Matriz de Inércia

CC = [Ct Z; Z Cr];   % Matriz de Coriolis

GG = [Gt; Gr];       % Vetor de Forças Gravitacionais

% Matriz de Acoplamento e Matriz dos Braços de Forcas
% [F1 F2 F3]' = R*At*[fx fy fz fytr]'
At = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];

% [L M N]' = Ar*[fx fy fz fytr]'
Ar = [ cParameters.k1  cParameters.k1 -cParameters.k1  -cParameters.k1;
    -cParameters.k1  cParameters.k1  cParameters.k1  -cParameters.k1;
    cParameters.k2 -cParameters.k2  cParameters.k2  -cParameters.k2];

A = [At;Ar];

% Matriz Pseudo-Inversa de A: A-sharp
As = pinv(A); %(A'*A)\A';

% Montagem da matriz sub-atuada ativa
% Matriz de Inérica
Ma = As*MM;
Map = Ma(:,1:2); % Passive variables X and Y
Maa = Ma(:,3:6); % Active variables Z, PHI, THETA and PSI

% Matriz de Coriolis e Vetor de Forças Gravitacionais Ativa
Ea  = As*(CC*drone.pPos.X(7:12) + GG);

% Escrita das matrizes passivas
MP = R'*Mt;
GP = R'*Gt;

Mpp =  MP(1:2,1:2);
Mpa = [MP(1:2,3) zeros(2,3)];

Ep = GP(1:2,1);

%==========================================================================
% Representação na forma sub-atuada
% M = [Mpp Mpa; Map Maa];
% E = [Ep; Ea];

D = Maa - Map*(Mpp\Mpa);
H = Ea - Map*(Mpp\Ep);

eta = [etaz; etap; etat; etas];

% Vetor de Forças de referência aplicado no referencial do veículo
Fr = D*eta + H;

% Verificando se forças sobre o referência do veículo
% ocorrem somente na direção Z
fTau = A*Fr;
drone.pSC.T = fTau(3:6);
% ------------------------------------
% Forçando valores possíveis: 30% do valor da gravidade
if real(fTau(3)) < 0
    fTau(3) = cParameters.m*cParameters.g*0.3;
end
% ------------------------------------

% Considerando a situação mais simples de que a força de propulsão
% solicitada aos motores é imediatamente atendida
% NÂO considera modelo da bateria
% Modelo Inverso do Atuador: Forças desejada nos propulsores
Fd = As*fTau;

% Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
for ii = 1:4
    if Fd(ii) < 0
        Fd(ii) = 0;
    end
end

% 1: Fr -> Wr
Wda = drone.pSC.Wd;
drone.pSC.Wd = sqrt(Fd/cParameters.Cf);

% 2: Wr -> V
Vr = -cParameters.Vo + cParameters.Jm*cParameters.R/cParameters.Km*(drone.pSC.Wd-Wda)/cParameters.Ts + ...
    (cParameters.Bm*cParameters.R/cParameters.Km + cParameters.Kb)*drone.pSC.Wd + ...
    cParameters.Ct*cParameters.R/cParameters.Km/cParameters.r*drone.pSC.Wd.^2;


% 3: V -> Xr
drone.pSC.Xr(4) = drone.pPos.X(4) + 1/(cParameters.kdp+cParameters.kpp*cParameters.Ts)*...
    (cParameters.kdp*(drone.pSC.Xr(4)-drone.pPos.X(4)) + 1/4*cParameters.Ts*([1 1 -1 -1]*Vr));

drone.pSC.Xr(5) = drone.pPos.X(5) + 1/(cParameters.kdt+cParameters.kpt*cParameters.Ts)*...
    (cParameters.kdt*(drone.pSC.Xr(5)-drone.pPos.X(5)) + 1/4*cParameters.Ts*([-1 1 1 -1]*Vr));

drone.pSC.Xr(9) = drone.pPos.X(9) + 1/(cParameters.kdz+cParameters.kpz*cParameters.Ts)*...
    (cParameters.kdz*(drone.pSC.Xr(9)-drone.pPos.X(9)) + 1/4*cParameters.Ts*([1 1 1 1]*Vr));

drone.pSC.Xr(12) = drone.pPos.X(12) + 1/(cParameters.kds+cParameters.kps*cParameters.Ts)*...
    (cParameters.kds*(drone.pSC.Xr(12)-drone.pPos.X(12)) + 1/4*cParameters.Ts*([1 -1 1 -1]*Vr));

% 4: Xr -> U
drone.pSC.Ud(1) =  drone.pSC.Xr(4) /cParameters.uSat(1);   % Phi
drone.pSC.Ud(2) = -drone.pSC.Xr(5) /cParameters.uSat(2);   % Theta
drone.pSC.Ud(3) =  drone.pSC.Xr(9) /cParameters.uSat(3);   % dZ
drone.pSC.Ud(4) = -drone.pSC.Xr(12)/cParameters.uSat(4);  % dPsi

drone.pSC.Ud = tanh(drone.pSC.Ud);

end
