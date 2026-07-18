function rSendControlSignals(drone)

    if drone.pFlag.Connected == 1
        if ~strcmp(drone.pCom.State,'landed')    % flying state flag (1=flying/0=landed)
            % Experiment Mode: Ardrone 2.0
            drone.rCommand;
        end
    else
        % Simulation Mode
        drone.pSC.U = [drone.pSC.Ud(1) drone.pSC.Ud(2) drone.pSC.Ud(3) drone.pSC.Ud(6)]';
        %drone.sDynamicModel;
        drone.sUAVmodel_LinearIdentified;
    end
    
    % Stand-by mode
    % disp(drone.pSC.Ud)
    drone.pSC.Ud = [0; 0; 0; 0; 0; 0];

end