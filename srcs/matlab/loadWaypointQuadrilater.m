numWaypoints = 5;
x_waypoints = [homeConfig(1)-50, homeConfig(1)-60, homeConfig(1)+70, homeConfig(1)+80, homeConfig(1)-50];
y_waypoints = [homeConfig(2)-200, homeConfig(2)-100, homeConfig(2)+100, homeConfig(2)+100, homeConfig(2)+100];
z_waypoints = [homeConfig(3)-50, homeConfig(3)+60, homeConfig(3)+70, homeConfig(3)-80, homeConfig(3)-50];

waypoints = zeros(3, numWaypoints);
for i = 1:numWaypoints
    waypoints(:, i) = [x_waypoints(i); y_waypoints(i); z_waypoints(i)];
end
fprintf('numel(waypoints) =\n'); disp(numWaypoints);

% create velocity matrix
waypointTimes = 0:numWaypoints:(numWaypoints-1)*numWaypoints;
ts = 0.8;
trajTimes = 0:ts:waypointTimes(end); % Trajectory sample time
% Boundary conditions (for polynomial trajectories)
waypointVels = 1.5 * ...
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 0 0]';
waypointAccels = zeros(size(waypointVels));

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', []);

clear x_waypoints y_waypoints z_waypoints delta ts