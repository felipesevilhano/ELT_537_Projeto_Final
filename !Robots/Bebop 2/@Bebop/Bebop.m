classdef Bebop < handle
    
    % Initial letters for properties or methods:
    %
    % i - initilialization scripts
    % p - drone parameters: modelling, controllers, communication
    % r - real drone commands or information
    % s - simulatated drone
    % m - matlab script functions   

    % - esta classe roda o Bebop com ROS, falta integrar o robô com conexão
    % direta no wi-fi do pc.

    properties
        
        % Properties or Parameters
        pCAD      % Bebop 3D image
        pPar      % Parameters Dynamic Model
        pID       % Identification
                
        % Control variables
        pPos      % Posture
        pSC       % Signals
        pFlag     % Flags        
                
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication   
        pCam  % Camera data
        pOdom % Odometry Data  
        Marker
        
        % ROS Parameters
        % Global Parametrs                
        pWorkSpaceName  % Standard 'bebop_ws'
        pNamespace      % Standard 'bebop'
        pTxtTopic
                       
        % Standards Messages        
        pVel
        pStdMsgEmpty        
        pStdMsgGeoTwi
        pBatteryLevel   % battery level
        pPosition       % UAV position collect from OptiTrack/Motive network
        pPose           % UAV posture collect from OptiTrack/Motive network
        pWayPoints      % Desired way-points calculated from Q-Learning Local Planning 
        pObstacles      % mensagem de obstáculos do OptiTrack

        % Standards Publishers
        pubCmdVel
        pubBrTakeoff
        pubBrLand
        pubBposition    % publica a posição corrente do UAV lida da rede OptiTrack/Motive
        pubBpose        % publica a postura corrente do UAV lida da rede OptiTrack/Motive
        pubObsPosition  % publica a postura corrente do UAV lida da rede OptiTrack/Motive
        pubOdomInit
        
        % Standards Listeners   
        subOdom
        subOdomLocal
        subOdomOpt
        subLand
        subBattery
        subMarker
        subWayPoints    % get the Q-Learning Local Planning Way-points for the path-planning
        
    end
    methods
        function obj = Bebop(ID,iNamespace)
            iFlags(obj);

            if nargin == 2 
                mInit(obj,ID,iNamespace); % Initialize variables
            elseif nargin < 2
                if nargin < 1
                    ID = 1;
                end
                obj.pFlag.Connected = 0;
            end   

            % Classic Drone Variables
            obj.pID = ID;

            iControlVariables(obj);
            iParameters(obj);

            mCADload(obj);

        end   
        
        function Callback_EmgStop(obj,~,~)
            obj.pFlag.EmergencyStop = 0;
        end

        % ==================================================
        % Initialization parameters
        iControlVariables(obj);
        iParameters(obj);
        
        % ==================================================
        % Bebop functions - ROS
        % Communication
        
                    
        % Takeoff/Landing
        rTakeOff(obj);        
        rLand(obj);                
        
        % Set Methods
        rSetGeometryMsg(obj,twistObj);
        rSetGeometryMsgVar(obj);        
        msgObj = mSetTwistObj(msgObj,Lx,Ly,Lz,Ax,Ay,Az);
                
                
        % Get Methods
        
                
        % Send Methods        
        rCmdStop(obj);
        rCmdVel(obj,iVar);                       
                
        % Send Commands
        rCommand(obj);
        rSetLed(obj,id,freq,duration);
        rSendControlSignals(obj);
                
        % Data request
        rGetSensorData(obj);   
        rGetSensorDataLocal(obj);
        rGetSensorDataOpt(obj);
		rGetLastSensorDataOpt(obj);
        rGetSensorOdomMsg(obj);
        rGetMarker(obj);
                             
        % ==================================================
        % obj functions
        % Communication
        rConnect(obj);
        rDisconnect(obj);        

        % Emergency
        rEmergency(obj)            
        
        % ==================================================
        % Bebop 3D Image
        mCADload(obj);
        mCADcolor(obj,cor);
        mCADplot(obj,visible);
        mCADdel(obj);
        
        % ==================================================
        % Bebop Models for simulation
        sDynamicModel(obj);
        sUAVmodel_LinearIdentified(obj);

         
    end
end