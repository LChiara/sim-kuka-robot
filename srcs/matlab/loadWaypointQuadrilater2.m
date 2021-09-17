numWaypoints = 6;
xH = homeConfig(1); yH = homeConfig(2); zH = homeConfig(3);
x_waypoints = [xH-50,   xH-60,  xH+70,  xH-50,  xH+80,  xH-50];
y_waypoints = [yH-200,  yH-100, yH+100, yH-200, yH+100, yH+100];
z_waypoints = [zH-50, 	zH+60,  zH+70,  zH-50,  zH-80,  zH-50];

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
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 0 0; 0 1 0]';
waypointAccels = zeros(size(waypointVels));

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes);

clear x_waypoints y_waypoints z_waypoints delta ts