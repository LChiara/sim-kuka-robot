function [tau, status] = solveInverseDynamic(dhParameters, M, Inertia, Fext, q, qd, qdd)

% calculate the joint forces and torques necessary to create
% the desired joint accelerations at the current joint position
% and velocities.

Rall = ones(3, 3, 6);   % initialize R
w = zeros(3, 1);        % initialize velocity
wd = zeros(3, 1);       % initialize acceleration
vd = [0; 0; -9.8];      % initialize linear acceleration with gravity
z0 = [0; 0; 1];
r = [-0.25; 0; 0];
status = cast(0, 'int8');

nn  = Fext(1:3);
f   = Fext(4:6);

% initizialize output of forward iterations
Fm = zeros(3, 6);
Nm = zeros(3, 6);

% forward iteration
for i=1:6
    dh = dhParameters(i, :);
    pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
    Ti = solver.computeT(dh, q(i));
    R = Ti(1:3, 1:3)';
    Rall(:, :, i) = R; % save R in order to reuse it in the next step
    
    % wd is the sum of wd(i-1) (expressed in frame i), and of a
    % term due to qdd(i) and of the velocity product term due
    % to wd and qd(i).
    wd = R*(wd+z0*qdd(i) + cross(w, z0*qd(i)));
    % w is the sum of w(i-1) (expressed in frame i), and of a
    % term due to qd(i).
    w = R*(w + z0*qd(i));
    vd = cross(wd, pStar) + cross(w, cross(w, pStar)) + R*vd;
    
    vHat = cross(wd, r) + cross(w, cross(w, r)) + vd;
    
    Fm(:, i) = M(i)*vHat;
    Nm(:, i) = Inertia(:, :, i)*wd + cross(w, Inertia(:, :, i)*w);
end

% backward iteration
tau = zeros(1, 6); % initialize output of backward iteration
for j=6:-1:1
    if j == 6
        R = eye(3);
    else
        R = Rall(:, :, j+1);
    end
    dh = dhParameters(i, :);
    pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
    %update nn
    nn = R*(nn + cross(R*pStar, f)) + cross(pStar + r, Fm(:, j)) + Nm(j);
    %upd
    f = R*f + Fm(:, j);
    tau(j) = nn'*(R*z0);
end
status = cast(1, 'int8');
end
