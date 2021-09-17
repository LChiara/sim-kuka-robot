% startup.m

if exist('config', 'var')
    TRAJECTORY = config.trajectory;
    TP_MODE = config.space;
else
    TRAJECTORY = 1;
    TP_MODE = 2;
end

switch TRAJECTORY
    case 0
        waypointLoaderScript = 'loadWaypointSimple';
    case 1
        waypointLoaderScript = 'loadWaypointQuadrilater';
    case 2
        waypointLoaderScript = 'loadWaypointQuadrilater2';
    case 3
        waypointLoaderScript = 'loadWaypointQuadrilater3';
end

if ~exist('IsSixAxesRobotInitialized', 'var')
    fprintf('Startup KukaKR6...\n');
    fprintf('Add Matlab paths...\n');
    addpath(fullfile('srcs'));
    addpath(fullfile('srcs\matlab'));
    addpath(fullfile('srcs\simulink'));
    addpath(fullfile('srcs\simulink\blockicons'));
    addpath(fullfile('srcs\wrl'));
    
    loadStr = 'Load';
else
    loadStr = 'Reload';
end

fprintf('%s D-H parameters...\n', loadStr);
d = [335; 0; 0; -295; 0; -80];
a = [75; 270; 90; 0; 0; 0];
alpha = [-pi/2; 0; pi/2; -pi/2; pi/2; pi];
offset = [0; 0; 0; 0; 0; 0];
dhParametersStruct = struct( ...
    'd', d, ...
    'a', a, ...
    'alpha', alpha, ...
    'offset', offset);

fprintf('%s Dynamic parameters...\n', loadStr);
loadDynamicParameters; % load dynamic parameters

fprintf('Creating SixAxesRobot...\n')
sixAxesRobot = SixAxesRobot(dhParametersStruct);
sixAxesRobot.Name = 'SixAxesRobot';
sixAxesRobot.M = dynamicParametersStruct.M;
sixAxesRobot.I = dynamicParametersStruct.I;
disp(sixAxesRobot);

fprintf('%s Waypoints from "%s"...\n', loadStr, waypointLoaderScript);
% get home configuration
q_config = [0 0 0 0 0 0]; T_homeConfig = sixAxesRobot.solveFK(q_config); homeConfig = T_homeConfig(1:3, 4);
% load waypoints, waypoints velocities and waypoints accelerations
run(waypointLoaderScript);

fprintf('%s Bus Objects...\n', loadStr);
load('busObjects.mat');

fprintf('Opening model "KukaSimModel.slx"...\n');
open_system('KukaSimModel.slx');
fprintf('Setting "StopTime"=%ds...\n', max(waypointsStruct.times));
set_param('KukaSimModel', 'StopTime', num2str(max(waypointsStruct.times)));
%dt_FixedStep = str2double(get_param('KukaSimModel', 'FixedStep'));
dt_FixedStep = 0.1;

if TP_MODE == 1
    modus = 'TaskSpace';
else
    modus = 'JointSpace';
end
fprintf('Setting Variant TP_MODE=%d->["%s"]\n', TP_MODE, modus);

IsSixAxesRobotInitialized = true;

clear i q_config T_homeConfig trajTimes waypointAccelTimes;
clear d a alpha offset dir loadStr;
clear waypoints waypointVels waypointAccels waypointTimes

disp('Initialization Done.');
