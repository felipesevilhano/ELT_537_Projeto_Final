function [drone,cGains,cParameters] = cArDrone_UnderActuated_NearHovering(drone,cGains,cParameters)

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

    % Rotor Parameters
    cParameters.r = 9; % Reduction Gear
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
    cParameters.uSat(1) = 45*pi/180;  % Max roll  angle reference
    cParameters.uSat(2) = 45*pi/180;  % Max pitch angle reference
    cParameters.uSat(3) = 2;         % Max altitude rate reference
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
    cGains = [1 1 2 5 5 1; 5 5 10 10 10 5];
    %cGains = [0.5 0.5 2 5 5 1; 2.5 2.5 10 10 10 2.5];
    % disp('Gains not given. Using standard ones.');
end


% -------------------------------------------------
% System dynamics
% Translational dynamics
MMt = [cParameters.m*cos(drone.pPos.X(6)), cParameters.m*sin(drone.pPos.X(6)), -cParameters.m*drone.pPos.X(5);...
    cParameters.m*(-sin(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*cos(drone.pPos.X(6))), cParameters.m*(cos(drone.pPos.X(6))+drone.pPos.X(4)*drone.pPos.X(5)*sin(drone.pPos.X(6))), cParameters.m*drone.pPos.X(4);...
    cParameters.m*(drone.pPos.X(5)*cos(drone.pPos.X(6))+drone.pPos.X(4)*sin(drone.pPos.X(6))), cParameters.m*(drone.pPos.X(5)*sin(drone.pPos.X(6))-drone.pPos.X(4)*cos(drone.pPos.X(6))), cParameters.m];

CCt = zeros(3,3);

GGt = [-drone.pPos.X(5)*cParameters.m*cParameters.g;...
    drone.pPos.X(4)*cParameters.m*cParameters.g;...
    cParameters.m*cParameters.g];

% Rotational dynamics
MMr = [cParameters.Ixx, 0, -cParameters.Ixx*drone.pPos.X(5);...
    0, cParameters.Iyy+cParameters.Izz*drone.pPos.X(4)^2, cParameters.Iyy*drone.pPos.X(4)-cParameters.Izz*drone.pPos.X(4);...
    -cParameters.Ixx*drone.pPos.X(5), cParameters.Iyy*drone.pPos.X(4)-cParameters.Izz*drone.pPos.X(4), cParameters.Ixx*drone.pPos.X(5)^2+cParameters.Iyy*drone.pPos.X(4)^2+cParameters.Izz];

CCr = [0, -cParameters.Izz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(-cParameters.Ixx-cParameters.Iyy+cParameters.Izz)*drone.pPos.X(12), 1/2*(-cParameters.Ixx-cParameters.Iyy+cParameters.Izz)*drone.pPos.X(11)-cParameters.Iyy*drone.pPos.X(4)*drone.pPos.X(12);...
    cParameters.Izz*drone.pPos.X(4)*drone.pPos.X(11)+1/2*(cParameters.Ixx+cParameters.Iyy-cParameters.Izz)*drone.pPos.X(12), cParameters.Izz*drone.pPos.X(4)*drone.pPos.X(10), 1/2*(cParameters.Ixx+cParameters.Iyy-cParameters.Izz)*drone.pPos.X(10)-cParameters.Ixx*drone.pPos.X(5)*drone.pPos.X(12);...
    1/2*(-cParameters.Ixx+cParameters.Iyy-cParameters.Izz)*drone.pPos.X(11)+(cParameters.Ixx*drone.pPos.X(5)+cParameters.Iyy*drone.pPos.X(4))*drone.pPos.X(12), 1/2*(-cParameters.Ixx+cParameters.Iyy-cParameters.Izz)*drone.pPos.X(10)+cParameters.Ixx*drone.pPos.X(5)*drone.pPos.X(12), cParameters.Iyy*drone.pPos.X(4)*drone.pPos.X(10)+cParameters.Ixx*drone.pPos.X(5)*drone.pPos.X(11)];

GGr = zeros(3,1);

MM = [MMt zeros(3,3); zeros(3,3) MMr];
CC = [CCt zeros(3,3); zeros(3,3) CCr];
GG = [GGt; GGr];

% -------------------------------------------------
% Describing the system as an under-actuaded one
MMpp = MM(1:2,1:2);
MMpa = MM(1:2,3:6);
MMap = MM(3:6,1:2);
MMaa = MM(3:6,3:6);

CCpp = CC(1:2,1:2);
CCpa = CC(1:2,3:6);
CCap = CC(3:6,1:2);
CCaa = CC(3:6,3:6);

GGpp = GG(1:2,1);
GGaa = GG(3:6,1);

EEa = CCaa*drone.pPos.dX(9:12) + GGaa + MMap*drone.pPos.dX(7:8) + CCap*drone.pPos.dX(1:2);

% -------------------------------------------------
% Under-actuated controller
% Gains

KKL1a  = diag(cGains(1,3:6));
KKL2a  = diag(cGains(2,3:6));
KKL3a  = 2*sqrt(KKL1a);
KKL4a  = 2*sqrt(KKL1a*KKL2a)/KKL3a;

KKL1p  = diag(cGains(1,1:2));
KKL2p  = diag(cGains(2,1:2));
KKL3p  = 2*sqrt(KKL1p);
KKL4p  = 2*sqrt(KKL1p*KKL2p)/KKL3p;


drone.pPos.Xtil = drone.pPos.Xd-drone.pPos.X;

etap = drone.pPos.dXd(1:2) + KKL1p*tanh(KKL2p*drone.pPos.Xtil(7:8))  + KKL3p*tanh(KKL4p*drone.pPos.Xtil(1:2));
etaa = drone.pPos.dXd(3:6) + KKL1a*tanh(KKL2a*drone.pPos.Xtil(9:12)) + KKL3a*tanh(KKL4a*drone.pPos.Xtil(3:6));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 02/11/2021: ASBrandao 
% Using absolute value of eta_z to compute the roll and pitch referecenes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

drone.pPos.Xd(10) = (-drone.pPos.Xd(4)+(atan2((etap(1)*sin(drone.pPos.X(6))-etap(2)*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(abs(etaa(1))+cParameters.g))))/cParameters.Ts;
drone.pPos.Xd(11) = (-drone.pPos.Xd(5)+(atan2((etap(1)*cos(drone.pPos.X(6))+etap(2)*sin(drone.pPos.X(6))),(abs(etaa(1))+cParameters.g))))/cParameters.Ts;

drone.pPos.Xd(4) = atan2((etap(1)*sin(drone.pPos.X(6))-etap(2)*cos(drone.pPos.X(6)))*cos(drone.pPos.X(5)),(abs(etaa(1))+cParameters.g));
drone.pPos.Xd(5) = atan2((etap(1)*cos(drone.pPos.X(6))+etap(2)*sin(drone.pPos.X(6))),(abs(etaa(1))+cParameters.g));

drone.pPos.Xtil = drone.pPos.Xd-drone.pPos.X;
etaa = drone.pPos.dXd(3:6) + KKL1a*tanh(KKL2a*drone.pPos.Xtil(9:12)) + KKL3a*tanh(KKL4a*drone.pPos.Xtil(3:6));

drone.pSC.T = MMaa*etaa + EEa;

% Matriz de Acoplamento e Matriz dos Braços de Forcas
A = [ 1 1 1 1;
     cParameters.k1  cParameters.k1 -cParameters.k1  -cParameters.k1;
    -cParameters.k1  cParameters.k1  cParameters.k1  -cParameters.k1;
     cParameters.k2 -cParameters.k2  cParameters.k2  -cParameters.k2];

Fd = A\drone.pSC.T;

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
drone.pSC.Ud(1) =  drone.pSC.Xr(4)/cParameters.uSat(1);   % Phi
drone.pSC.Ud(2) = -drone.pSC.Xr(5)/cParameters.uSat(2);   % Theta
drone.pSC.Ud(3) =  drone.pSC.Xr(9)/cParameters.uSat(3);   % dZ
drone.pSC.Ud(4) = -drone.pSC.Xr(12)/cParameters.uSat(4);  % dPsi

drone.pSC.Ud = tanh(drone.pSC.Ud);


end