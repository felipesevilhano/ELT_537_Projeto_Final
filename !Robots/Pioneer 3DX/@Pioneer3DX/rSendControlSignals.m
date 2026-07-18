function rSendControlSignals(p3dx,model)

if p3dx.pFlag.Connected == 1
    % Experiment Mode: MobileSim or P3DX
    arrobot_setvel(p3dx.pSC.Ud(1)*1000);      % Linear velocity
    arrobot_setrotvel(p3dx.pSC.Ud(2)/pi*180); % Angular velocity
    
    % Stand-by mode
    p3dx.pSC.Ud = [0; 0];
else
    % Simulation Mode
    switch model
        case 'kinematic'
            p3dx.pSC.U = p3dx.pSC.Ud; 
            p3dx.sKinematicModel; 
        case 'extended kinematic'
            p3dx.pSC.U = p3dx.pSC.Ud; 
            p3dx.sExtendedKinematicModel; 
        case 'dynamic'
            p3dx.sDynamicModel;
        otherwise
            disp('Unidentified robot model')
    end

    % Stand-by mode
    p3dx.pSC.Ud = [0; 0];
end


