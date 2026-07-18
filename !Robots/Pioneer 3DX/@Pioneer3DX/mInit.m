% Initialize variables
function mInit(obj,iNamespace)
    
    obj.pFlag.Connected = 1;
    obj.pFlag.isTracked = 1;
    
    % ROS inicialize Variables
    % Inicialize Variables        
    obj.pWorkSpaceName = 'RosAria'; 
    obj.pNamespace     = iNamespace;      
    
    
    % Inicialize Messages Txt Types            
    obj.pTxtTopic.OdomOpt   = strcat('/',obj.pNamespace,'/opt_odom');    % '/RosAria/new_odom';    
    obj.pTxtTopic.Echo      = 'echo';  
    
    % Inicialize Messages    
    obj.pOdom        = rosmessage('nav_msgs/Odometry');       
    obj.pVel         = rosmessage('geometry_msgs/Twist');
    obj.pPot1         = rosmessage('std_msgs/Float64');
    obj.pPot2         = rosmessage('std_msgs/Float64');
    
    % Inicialize Publishers    
    obj.pubCmdVel    = rospublisher(strcat('/',obj.pNamespace,'/cmd_vel')); 
    
    % Inicialize Subscribers
    obj.subOdomOpt   = rossubscriber(obj.pTxtTopic.OdomOpt,'nav_msgs/Odometry');
    obj.subOdom      = rossubscriber(strcat('/',obj.pNamespace,'/pose'),'nav_msgs/Odometry');
    
end

