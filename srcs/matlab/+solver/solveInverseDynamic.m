function [tau, status] = solveInverseDynamic(dhParameters, M, Inertia, Fext, q, qd, qdd)
% calculate the joint forces and torques necessary to create
% the desired joint accelerations at the current joint position
% and velocities.

G = [0; 0; -9.8];   % gravity vector
R_ = ones(3, 3, 6); % initialize R
w  = zeros(3, 6);   % initialize angular velocity
wd = zeros(3, 6);   % initialize angular acceleration
vd = zeros(3, 6);   % initialize linear acceleration
vd(:, 1) = G;       % initialize linear acceleration with gravity

z0 = [0; 0; 1];
r  = [-0.25; 0; 0]; % distance to the center of mass

nn = Fext(1:3);    % initialize nn (backward iteration)
f  = Fext(4:6);    % initialize f  (backward iteration)

tau = zeros(1, 6);  % initialize output

for i=1:6
    dh = dhParameters(i, :);
    pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
    Ti = solver.computeT(dh, q(i));
    R = Ti(1:3, 1:3)';
    R_(:, :, i) = R; % save R in order to reuse it in the next step
    
    if i == 1
        wd(:, 1) = R*(z0*qdd(i));
        w(:, 1)  = R*(z0*qd(i));
        vd(:, i) = cross(wd(:, i), pStar) + ...
            cross(w(:, i), cross(w(:, i), pStar)) + ...
            R*G; % use G for the first iteration
    else
        % wd is the sum of wd(i-1) (expressed in frame i), and of a
        % term due to qdd(i) and of the velocity product term due
        % to wd and qd(i).
        wd(:, i) = R*(wd(:, i-1)+z0*qdd(i) + cross(w(:, i-1), z0*qd(i)));
        % w is the sum of w(i-1) (expressed in frame i), and of a
        % term due to qd(i).
        w(:, i)  = R*(w(:, i-1) + z0*qd(i));
        vd(:, i) = cross(wd(:, i), pStar) + ...
            cross(w(:, i), cross(w(:, i), pStar)) + ...
            R*vd(:, i-1);
    end
end

% backward iteration
for j=6:-1:1
    % calculate total force F on the center of mass (Newton)
    vHat = cross(wd(:, j), r) + ...
        cross(w(:, j), cross(w(:, j), r)) + vd(:, j);
    linkFtot = M(j)*vHat;
    % calculate total torque tau about the center of mass (Euler)
    linkTauTot = Inertia(:, :, j)*wd(:, j) + cross(w(:, j), Inertia(:, :, j)*w(:, j));
    
    if j == 6
        R = eye(3);
    else
        R = R_(:, :, j+1)';
    end
    dh = dhParameters(i, :);
    pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
    % update nn for the next iteration
    nn = R*(nn + cross(R'*pStar, f)) + ...
        cross(pStar + r, linkFtot) + linkTauTot;
    % update f for the next iteration
    f = R*f + linkFtot;
    tau(j) = nn'*(R'*z0);
end
status = cast(1, 'int8');
end
