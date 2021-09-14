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

fprintf('%s Waypoints...\n', loadStr);
% get home configuration
q_config = [0 0 0 0 0 0]; T_homeConfig = sixAxesRobot.solveFK(q_config); homeConfig = T_homeConfig(1:3, 4);
% load waypoints, waypoints velocities and waypoints accelerations
loadWaypoints;
waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes);

fprintf('%s Bus Objects...\n', loadStr);
load('busObjects.mat');

fprintf('Opening model "KukaSimModel.slx"...\n');
open_system('KukaSimModel.slx');
dt_FixedStep = str2double(get_param('KukaSimModel', 'FixedStep'));

TP_MODE = 1;
fprintf('Setting Variant TP_MODE=%d\n', TP_MODE);

IsSixAxesRobotInitialized = true;

clear i q_config T_homeConfig trajTimes waypointAccelTimes;
clear d a alpha offset dir loadStr;
%clear waypoints waypointVels waypointAccels waypointTimes <- still used in
%variant

disp('Initialization Done.');
