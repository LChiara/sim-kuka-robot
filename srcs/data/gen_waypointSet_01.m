function waypointsStruct = gen_waypointSet_01(homeConfig)
% create waypoints
numWaypoints = 5;
x_waypoints = [homeConfig(1)-50, homeConfig(1)-60, homeConfig(1)+70, homeConfig(1)+80, homeConfig(1)-50];
y_waypoints = [homeConfig(2)-200, homeConfig(2)-100, homeConfig(2)+100, homeConfig(2)+100, homeConfig(2)+100];
z_waypoints = [homeConfig(3)-50, homeConfig(3)+60, homeConfig(3)+70, homeConfig(3)-80, homeConfig(3)-50];
waypoints = zeros(3, numWaypoints);
for i = 1:numWaypoints
    waypoints(:, i) = [x_waypoints(i); y_waypoints(i); z_waypoints(i)];
end
% create orientations
orientations = [];
% create waypoint times (reached waypoint instant)
waypointTimes = 0:numWaypoints:(numWaypoints-1)*numWaypoints;
% define velocity boundaries
waypointVels = 1.5 * ...
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 0 0]';
% define accelerations boundaries
waypointAccels = zeros(size(waypointVels));

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', orientations);
end