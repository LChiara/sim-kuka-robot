function [q_out, qd_out, qdd_out] = solveForwardKinematics(dhParameters, q, dt)
coder.internal.errorIf(nargin > 2 & nargout < 3, 'solver:solveForwardKinematics:NotEnoughInputArgument');
n = 6;
y = zeros(3, n);
T = eye(4, 4); % initialize T for first computation
for i=1:n
    T = T*solver.computeT(dhParameters(i, :), q(i));
    y(:, i) = T(1:3, 4);
end
if nargout > 1 && nargin > 2 % if dt is given and gradient can be calculated
    q_out = y;
    qd_out = gradient(q_out, dt);
    qdd_out = gradient(qd_out, dt);
else
    q_out = T;
    qd_out = zeros(3, n);
    qdd_out = zeros(3, n);
end
end

