function plotWaypoints_(waypointsRaw, trajectoryRaw, trajectoryColor, f)
if nargin < 2
    trajectoryRaw = [];
    trajectoryColor = [];
end
if nargin < 3
    trajectoryColor = 'k';
end

if nargin < 4
    f = gcf;
end
if ~isempty(f)
    clf('reset');
end
hold on; % ensure hold on

% rotate for plot purposes
R = [0 0 1; 0 1 0; -1 0 0];
waypoints = R*waypointsRaw;

% axes configuration (limit and view)
xlabel('X'); ylabel('Y'); zlabel('Z');
getAxisLim = @(x) [min(x)-abs(floor(min(x)/10)), max(x)+abs(floor(max(x)/10))];
xlim(getAxisLim(waypoints(1, :)));
ylim(getAxisLim(waypoints(2, :)));
zlim(getAxisLim(waypoints(3, :)));
view(105, 26)

% plot waypoints and label
delta = 5;
for i=1:size(waypoints, 2)
    text(waypoints(1, i)+delta, ...
        waypoints(2, i)+delta, ...
        waypoints(3, i)+delta, ...
        sprintf('P%d', i));
end
scatter3(waypoints(1, :), ...
    waypoints(2, :), ...
    waypoints(3, :), ...
    75, "black", 'LineWidth', 1);
scatter3(waypoints(1, :), ...
    waypoints(2, :), ...
    waypoints(3, :), ...
    '.', "black", 'LineWidth', 1.5);
% plot expected trajectory that connects waypoints
plot3(waypoints(1, :), ...
    waypoints(2, :), ...
    waypoints(3, :), ...
    'k--');

% plot trajectory if requested
if ~isempty(trajectoryRaw)
    trajectory = R*trajectoryRaw; % rotate it for plot purpose as done for waypoints
    plot3(trajectory(1, :), ...
        trajectory(2, :), ...
        trajectory(3, :), ...
        "Color", trajectoryColor, "LineWidth", 1.5);
end
end