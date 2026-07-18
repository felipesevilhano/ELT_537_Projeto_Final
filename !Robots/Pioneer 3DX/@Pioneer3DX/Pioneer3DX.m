classdef Pioneer3DX < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        
        % Properties or Parameters
        pCAD   % Pioneer 3DX 3D image
        pPar   % Parameters
        pID    % Identification
        vrep   % V-Rep library
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
        pOdom % Odometry Data 
        pPot1 % Potentiometer Data
        pPot2 % Potentiometer Data

        % Status variables
        pMotors
        
        % ROS Parameters        
        % Global Parametrs                
        pWorkSpaceName  % Standard 'bebop_ws'
        pNamespace
        iNamespace
        pTxtTopic        
                       
        % Standards Messages        
        pVel
        pStdMsgEmpty        
        pStdMsgGeoTwi
        
        % Standards Publishers
        pubCmdVel
        
        % Standards Listeners   
        subOdom   
        subOdomOpt
        subSonar
        subPot1
        subPot2
        
        % Services
        serEnaMotor
        serDisMotor

    end
    
    methods
        function p3dx = Pioneer3DX(ID,iNamespace)
            iFlags(p3dx);

            if nargin == 2 
                mInit(p3dx,iNamespace);
            elseif nargin < 2
                ID = 1;
                p3dx.pFlag.Connected = 0;
            end                                    
            p3dx.pID = ID;

            iControlVariables(p3dx);
            iParameters(p3dx);
            mCADload(p3dx);
            mCADmake(p3dx);      
        end
        
        % ==================================================
        iControlVariables(p3dx);
        iParameters(p3dx);
        iFlags(p3dx);       
        
        % ==================================================
        % Pioneer 3DX 3D Image
        mCADload(p3dx);
        mCADmake(p3dx);
        mCADplot(p3dx);
        mCADdel(p3dx);

        mCADplot2D(p3dx,visible);
        mCADcolor(p3dx,color);
        
        % ==================================================
        % Pose definition, based on kinematic or dynamic model
        sKinematicModel(p3dx);          % (Ok)
        sInvKinematicModel(p3dx,dXr);
        
        sExtendedKinematicModel(p3dx);
        %sInvExtendedKinematicModel(p3dx);
        
        sDynamicModel(p3dx);
        sInvDynamicModel(p3dx);
        
        % ==================================================
        % Robot functions
        % Communication
        rConnect(p3dx);
        rDisconnect(p3dx);
        rSetPose(p3dx,Xo);

        % Data request
        rSendControlSignals(p3dx,model);
        rGetSensorDataOpt(p3dx);
        dist = rGetSonarData(p3dx);
        
        % Command
        % rSendControlSignals(obj);
        rCmdStop(p3dx);
        rCmdVel(p3dx);
        rCommand(p3dx);
        rEnableMotors(p3dx);
        rDisableMotors(p3dx);

        % ==================================================
      
    end
end