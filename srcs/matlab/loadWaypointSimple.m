numWaypoints = 10;
delta = 50;
x_waypoints = linspace(homeConfig(1)-delta, homeConfig(1)+delta, numWaypoints);
y_waypoints = linspace(homeConfig(2)-delta, homeConfig(2)+delta, numWaypoints);
z_waypoints = linspace(homeConfig(3)-delta, homeConfig(3)+delta, numWaypoints);

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
waypointVels = 0.1 * ...
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 0 1; 0 0 1; -1 0 0; 0 0 -1; 1 0 0; 0 0 1]';
waypointAccels = zeros(size(waypointVels));
waypointAccelTimes = diff(waypointTimes)/4; % Acceleration times (trapezoidal only)

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', []);

clear x_waypoints y_waypoints z_waypoints delta ts