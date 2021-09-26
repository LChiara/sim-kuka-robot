% startup.m

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

if exist('config', 'var')
    config__TRAJECTORY = config.trajectory;
    config__SPACE = config.space;
else
    config__TRAJECTORY = 5;
    config__SPACE = enum.SpaceEnum.JointSpace;
end
genWaypointsScript = str2func(sprintf('gen_waypointSet_%02d', config__TRAJECTORY));

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
waypointsStruct = loadWaypoints(config__TRAJECTORY, eeHomeConfig);

fprintf('-> %s Bus Objects...\n', loadStr);
load('busObjects.mat');

fprintf('-> Setting Variant Subsystem...\n');
TP_MODE = config__SPACE.double;
fprintf('TP_MODE=%d->["%s"]\n', TP_MODE, config__SPACE.char);

fprintf('-> Loading model "KukaSimModel.slx"...\n');
modelFileName = 'KukaSimModel';
load_system(modelFileName);
%dt_FixedStep = str2double(get_param('KukaSimModel', 'FixedStep'));
dt_FixedStep = 0.1;

IsSixAxesRobotInitialized = true;

clear i q_config T_homeConfig trajTimes waypointAccelTimes;
clear dir loadStr;
clear waypoints waypointVels waypointAccels waypointTimes;

disp('Initialization Done.');
