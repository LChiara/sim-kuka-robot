function waypointsStruct = gen_waypointSet_03(homeConfig)
% create waypoints
numWaypoints = 5;
xH = homeConfig(1); yH = homeConfig(2); zH = homeConfig(3);
x_waypoints = [xH-50,   xH+80,  xH-60,  xH+70,  xH-50];
y_waypoints = [yH-200,  yH+100, yH-100, yH+100, yH-200];
z_waypoints = [zH-50, 	zH-80,  zH+60,  zH+70,  zH-50];
waypoints = zeros(3, numWaypoints);
for i = 1:numWaypoints
    waypoints(:, i) = [x_waypoints(i); y_waypoints(i); z_waypoints(i)];
end
% create orientations
orientations = [-pi/2, 0, 0;
    pi/4, 0, 0;
    0, -pi/2, 0;
    0, 0, pi/4;
    0, 0, pi/2]';
% create waypoint times (reached waypoint instant)
waypointTimes = 0:numWaypoints:(numWaypoints-1)*numWaypoints;
% define velocity boundaries
waypointVels = 0.5 * ...
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 1 0]';
% define accelerations boundaries
waypointAccels = zeros(size(waypointVels));

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', orientations);
end