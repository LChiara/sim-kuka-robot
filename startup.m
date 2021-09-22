% startup.m

if exist('config', 'var')
    TRAJECTORY = config.trajectory;
    TP_MODE = config.space;
else
    TRAJECTORY = 1;
    TP_MODE = 2;
end
genWaypointsScript = str2func(sprintf('gen_waypointSet_%02d', TRAJECTORY));

fprintf('============================\n');
fprintf('|    Startup KukaKR6...    |\n');
fprintf('============================\n');

if ~exist('IsSixAxesRobotInitialized', 'var')
    fprintf('-> Adding Matlab paths...\n');
    addpath(fullfile('.'));
    addpath(fullfile('srcs'));
    addpath(fullfile('srcs\data'));
    addpath(fullfile('srcs\matlab'));
    addpath(fullfile('srcs\simulink'));
    addpath(fullfile('srcs\simulink\blockicons'));
    addpath(fullfile('srcs\wrl'));
    
    loadStr = 'Loading';
else
    loadStr = 'Reloading';
end

fprintf('-> %s D-H parameters...\n', loadStr);
dhParametersStruct = loadDHParams();
dhParams = struct2table(dhParametersStruct);
disp(dhParams);


fprintf('-> %s Dynamic parameters...\n', loadStr);
dynamicParametersStruct = loadDynamicParams();
disp('Mass: ');
disp(dynamicParametersStruct.M);

fprintf('-> %s Waypoints...\n', loadStr);
fprintf('Computing home configuration...\n');
% get home configuration
qHomeConfig = [0 0 0 0 0 0];
THomeConfig = solver.solveForwardKinematics(dhParams, qHomeConfig);
eeHomeConfig = THomeConfig(1:3, 4);
% load waypoints, waypoints velocities and waypoints accelerations
waypointsStruct = loadWaypoints(TRAJECTORY, eeHomeConfig);

fprintf('-> %s Bus Objects...\n', loadStr);
load('busObjects.mat');

if TP_MODE == 1
    modus = 'TaskSpace';
else
    modus = 'JointSpace';
end
fprintf('-> Setting Variant Subsystem...\n');
fprintf('TP_MODE=%d->["%s"]\n', TP_MODE, modus);

fprintf('-> Opening model "KukaSimModel.slx"...\n');
open_system('KukaSimModel.slx');
fprintf('Setting "StopTime"=%ds...\n', max(waypointsStruct.times));
set_param('KukaSimModel', 'StopTime', num2str(max(waypointsStruct.times)));
%dt_FixedStep = str2double(get_param('KukaSimModel', 'FixedStep'));
dt_FixedStep = 0.01;

IsSixAxesRobotInitialized = true;

clear i q_config T_homeConfig trajTimes waypointAccelTimes;
clear dir loadStr;
clear waypoints waypointVels waypointAccels waypointTimes;

disp('Initialization Done.');
