function sUAVmodel_LinearIdentified(drone)

% Dynamic model from
% Pinto, A. O., Marciano, H. N., Bacheti, V. P., Moreira, M. S. M.,
% Brandão, A. S., & Sarcinelli-Filho, M. (2020, September).
% High-level modeling and control of the Bebop 2 micro aerial vehicle.
% In 2020 International Conference on Unmanned Aircraft Systems (ICUAS)
% (pp. 939-947). IEEE.
% https://doi.org/10.1109/ICUAS48674.2020.9213941


% Simulate ArDrone dynamic model
%
%      +------------+
% U -> | Identified | ->  X
%      | Model      |
%      +------------+
%

% 1: Receive input signal
%     pitch          | [-1,1] <==> [-5,5] degrees
%     roll           | [-1,1] <==> [-5,5] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s

drone.pPar.Xra = drone.pPar.Xr;

drone.pPar.Xr(4)  =  drone.pSC.Ud(1)*drone.pPar.uSat(1);
drone.pPar.Xr(5)  = -drone.pSC.Ud(2)*drone.pPar.uSat(2);
drone.pPar.Xr(9)  =  drone.pSC.Ud(3)*drone.pPar.uSat(3);
drone.pPar.Xr(12) = -drone.pSC.Ud(4)*drone.pPar.uSat(4);

F1 = [0.8417 0 0 0;
      0 0.8354 0 0;
      0 0 3.966 0;
      0 0 0 9.8524];

F2 = [0.18227 0 0 0;
      0 0.17095 0 0;
      0 0 4.001 0;
      0 0 0 4.7295];

R = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0 0;
     sin(drone.pPos.X(6))  cos(drone.pPos.X(6)) 0 0;
              0                     0           1 0;
              0                     0           0 1];

% - Model
drone.pPos.Xa = drone.pPos.X;
drone.pPos.ddXa = drone.pPos.ddX;

u = drone.pSC.U;
drone.pPos.dX = [drone.pPos.X(7:9)' drone.pPos.X(12)];

drone.pPos.ddX = R*F1*u - R*F2*drone.pPos.dX';


%--------------------------------------------
% Numerical integration of rotational movement
ddX = [drone.pPos.ddX(1:3)' 0 0 drone.pPos.ddX(4)]';
% ddXa = [drone.pPos.ddXa(1:3) 0 0 drone.pPos.ddXa(4)];
drone.pPos.X(7:12) = (ddX)*drone.pPar.Ts + drone.pPos.X(7:12);

% Position: Numerical integration [m]
%drone.pPos.X(1:2) = drone.pPos.X(7:8)*drone.pPar.ts + drone.pPos.X(1:2);

% Bebop pose - Numerical integration
for ii = 1:6
    drone.pPos.X(ii) = drone.pPos.X(ii+6)*drone.pPar.Ts + drone.pPos.X(ii);
    if ii > 3
        if drone.pPos.X(ii) > pi
            drone.pPos.X(ii) = -2*pi + drone.pPos.X(ii);
        end
        if drone.pPos.X(ii) < -pi
            drone.pPos.X(ii) = 2*pi + drone.pPos.X(ii);
        end
    end
end