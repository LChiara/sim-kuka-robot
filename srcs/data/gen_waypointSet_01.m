function waypointsStruct = gen_waypointSet_01(homeConfig)
%      _____+y
%     /|
%    / |
% z+/  |
%     +x
% ---------- %
% DEFINITION %
% ---------- %
% create waypoints
xH = homeConfig(1); yH = homeConfig(2); zH = homeConfig(3);
waypoints = [
    xH, yH, zH;
    xH-250, yH, zH;
    xH-250, yH-500, zH; %xH-100, yH-950, zH+700;
    xH+50,  yH+100, zH; %xH-150, yH-450, zH+600;
    xH+100, yH+200, zH;
    xH+100, yH-100, zH;
    xH, yH, zH;
    ]';
numWaypoints = size(waypoints, 2);

% create orientations
orientations = [
    0, 0, 0;
    pi, -pi/4, 0; %pi/2, 0, pi/2;
    pi, 0, 0; %pi/2, 0, pi/2;
    0, 0, 0;
    pi, 0, pi/8;
    pi, -pi/2, pi/8;
    0, 0, 0]';
%orientations = zeros(size(waypoints));
% create waypoint times (reached waypoint instant)
step = 5;
waypointTimes = 0:step:(numWaypoints-1)*step;
% define velocity boundaries
% waypointVels = zeros(size(waypoints));
waypointVels = 0.25 * ...
    [0  0  0; 
    -1  0  0; 
     0 -1  1; 
     1  1  0; 
     1  1  0; 
     0 -1  0; 
     0  0  0]';
% define accelerations boundaries
waypointAccels = zeros(size(waypointVels));

% ---------- %
% CHECK-SIZE %
% ---------- %
% check waypoint times dimension
if numWaypoints ~= numel(waypointTimes)
    warning('WaypointTimes length [%d] differs from number of Waypoints [%d]', ...
        numel(waypointTimes), numWaypoints);
end
% check waypoint velocities dimension
sizeWay = size(waypoints);
sizeVel = size(waypointVels);
if any(sizeWay ~= sizeVel)
    sizeVelC = num2cell(sizeVel);
    sizeWayC = num2cell(sizeWay);
    error('WaypointVels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
        sizeVelC{:}, sizeWayC{:});
end
% check waypoint acceleration dimension
sizeAcc = size(waypointAccels);
if any(size(waypointVels) ~= size(waypoints))
    sizeAccC = num2cell(size(sizeAcc));
    sizeWayC = num2cell(size(waypoints));
    error('WaypointAccels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
        sizeAccC{:}, sizeWayC{:});
end

% ---------- %
%   RETURN   %
% ---------- %
waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', orientations);
end