function plotWaypoints(waypoints, f)
if nargin < 2
    f = gcf;
end
if ~isempty(f)
    clf('reset');
end

hold on;
% plot waypoints
delta = 5;
for i=1:size(waypoints, 2)
    text(waypoints(1, i)+delta, ...
        waypoints(3, i)+delta, ...
        waypoints(2, i)+delta, ...
        sprintf('P%d', i));
end
scatter3(waypoints(1, :), ...
    waypoints(3, :), ...
    waypoints(2, :), ...
    75, "black", 'LineWidth', 1);
scatter3(waypoints(1, :), ...
    waypoints(3, :), ...
    waypoints(2, :), ...
    '.', "black", 'LineWidth', 1.5);

plot3(waypoints(1, :), ...
    waypoints(3, :), ...
    waypoints(2, :), ...
    'k--');

xlabel('X'); ylabel('Z'); zlabel('Y');
xlim([min(waypoints(1, :)), max(waypoints(1, :))]);
zlim([min(waypoints(2, :)), max(waypoints(2, :))]);
view([1 1 -1]);
end