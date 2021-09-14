classdef SixAxesRobot < handle
    %SIXAXESROBOT
    %
    %#codegen
    
    properties
        %general properties
        Name='GeneralRobot';    %name of the six axes robot
        DH      table;          %table describing the D-H parameters
        
        %kineamtic properties
        Tolerance double {mustBeNonnegative}; %tolerance used in IK solver
        
        %dynamic properties
        M       double {mustBeNonnegative}; % mass vector (Nx1) where N is the number of joint
        I       double; % intertia matrix (3x3xN) where N is the number of joint
        Fext    double; % external forces vector (Nx1) where N is the number of joint
    end
    
    properties(Constant)
        N = 6; %number of joints
    end
    
    properties(Access=private)
        Verbose logical;
        %InfoDate char; %creation time and date
    end
    
    methods
        function obj = SixAxesRobot(dh, name)
            if nargin == 2
                obj.Name = name;
            end
            
            validateattributes(dh,   {'table', 'struct'}, {'nonempty'});
            if isstruct(dh)
                obj.DH = struct2table(dh);
            else
                obj.DH = dh;
            end
            
            obj.Tolerance = 1e-6; %initialize tolerance
            obj.Verbose = true;
            
            obj.M = zeros(obj.N, 1);
            obj.I = zeros(3, 3, obj.N);
            obj.Fext = zeros(obj.N, 1);
            %obj.InfoDate = datetime(now, 'ConvertFrom', 'datenum');
        end
        
        function enableVerbose(obj)
            obj.Verbose = true;
        end
        
        function disableVerbose(obj)
            obj.Verbose = false;
        end
        
        % override
        function disp(obj)
            disp('=====================================');
            fprintf('Kuka 6-DOF: "%s"\n', obj.Name);
            disp('-------------------------------------');
            disp('DH Parameters: '); disp(obj.DH);
            disp('Masses: '); disp(obj.M);
            disp('=====================================');
        end
        
        %override
        function plot(obj, q)
            gcf; hold on;
            [~, linkPositions] = obj.solveFK(q);
            for i=1:obj.N
                if i > 1
                    plot3(linkPositions(1, [i-1, i]), ...
                        linkPositions(2, [i-1, i]), ...
                        linkPositions(3, [i-1, i]), ...
                        'LineWidth', 3, 'Color', '#FF9933');
                end
                scatter3(linkPositions(1, i), ...
                    linkPositions(2, i), ...
                    linkPositions(3, i), ...
                    75, ...
                    'MarkerEdgeColor', 'k', ...
                    'MarkerFaceColor', '#FF9933');
                text(linkPositions(1, i)+5, ...
                    linkPositions(2, i)+5, ...
                    linkPositions(3, i)+5, ...
                    sprintf('J%d', i), 'Fontsize', 10);
            end
            view([1 1 1]); % set axes orientation
            xlim([-500, 500]);
            ylim([-100, 100]);
            zlim([-100, 600]);
        end
        
        function [T, linkPositions] = solveFK(obj, q)
            T = eye(4, 4);
            if nargout > 1
                n = 3; %[x;y;z];
                linkPositions = zeros(n, obj.N);
            end
            for i=1:obj.N
                T = T*obj.computeT(obj.DH(i, :), q(i));
                if nargout > 1
                    linkPositions(:, i) = T(1:3, 4);
                end
            end
        end
        
        function [q, status] = solveIK(obj, T, maxIterations)
            if nargin < 3
                maxIterations = 50;
            end
            
            lambda  = 0.1;               %damping parameter.
            W       = eye(obj.N, obj.N); %W=weighting matrix, diagonal.
            Id       = eye(obj.N, obj.N); %Identity matrix
            
            q = zeros(1, obj.N);    %initialize solution q
            rejected = 0;           %initialize number of solutions
            status = int8(0);       %initialize flag for solution to failure
            
            for i=1:maxIterations
                % compute error
                yErr = obj.computeError(q, T);
                
                status = int8(obj.isSolutionFound(W, yErr));
                if status
                    break;
                end
                
                % compute the Jacobian
                J = obj.computeJacobian(q);
                
                %Gauss-Newton with Levenberg-Marquadt
                %source: https://people.duke.edu/~hpgavin/ce281/lm.pdf
                %"""
                %The Levenberg-Marquardt algorithm adaptively varies the
                %parameter updates between the gradient descent update and
                %the Gauss-Newton update:
                % [Jt*W*J + λ*I]*h = Jt*W(y-y^)
                % where h=parameter update.
                %- if λ small: Gauss-Newton update
                %- if λ large: gradient descent update
                %"""
                h = (J'*W*J + lambda*Id)\(J'*W*yErr);
                % compute q basing on the parameter update h
                qUpdate = q + transpose(h);
                % compute new error
                yErrNew = obj.computeError(qUpdate, T);
                
                if obj.calculateNormError(W, yErrNew) < obj.calculateNormError(W, yErr)
                    % as the solution improves, \lambda is decreased: the
                    % Levenberg-Marquardt method approaches the
                    % Gauss-Newthon method, and the solution typically
                    % accelerates to the local minimum.
                    q = qUpdate;
                    lambda = lambda/2;
                else
                    rejected = rejected + 1;
                    lambda = lambda*2;
                    if rejected == maxIterations
                        status = int8(2); % update status when all the solution have been rejected
                    end
                end
            end
            
            if obj.Verbose
                %obj.returnStatus(status, q, i);
            end
        end
        
        function tau = solveID(obj, q, qd, qdd)
            % calculate the joint forces and torques necessary to create
            % the desired joint accelerations at the current joint position
            % and velocities.
            
            Rall = ones(3, 3, obj.N); % initialize R
            w = zeros(3, 1);    % initialize velocity
            wd = zeros(3, 1);   % initialize acceleration
            vd = [0 0 -9.8];    % initialize linear acceleration with gravity
            z0 = [0 0 1];
            r = [-0.25 0 0];
            
            % initizialize output of forward iterations
            Fm = zeros(3, obj.N);
            Nm = zeros(3, obj.N);
            % forward iteration
            for i=1:obj.N
                dh = obj.DH(i, :);
                pStar = [dh.a, dh.d*cos(dh.alpha), dh.d*sin(dh.alpha)]; % move from i-1 to i
                Ti = obj.computeT(dh, q(i));
                R = Ti(1:3, 1:3)';
                Rall(:, :, i) = R; % save R in order to reuse it in the next step
                
                % wd is the sum of wd(i-1) (expressed in frame i), and of a
                % term due to qdd(i) and of the velocity product term due
                % to wd and qd(i).
                wd = R*(wd+z0*qdd(i) + cross(w, z0*qd(i)));
                % w is the sum of w(i-1) (expressed in frame i), and of a
                % term due to qd(i).
                w = R*(w + zo*qd(i));
                vd = cross(wd, pStar) + cross(w, cross(w, pStar)) + R*vd;
                
                vHat = wd*r' + cross(w, cross(w, r')) + vd;
                
                Fm(:, i) = obj.M(i)*vHat;
                Nm(:, i) = obj.I(:, :, i)*wd + w*obj.I(:, :, i)*w;
            end
            
            % backward iteration
            nn  = obj.F(1:3);
            f   = obj.F(4:6);
            % initialize output of backward iterations
            tau = zeros(obj.N, 1);
            for j=obj.N:-1:1
                if j == obj.N
                    R = eye(3);
                else
                    R = Rall(j+1);
                end
                dh = obj.DH(i, :);
                pStar = [dh.a, dh.d*cos(dh.alpha), dh.d*sin(dh.alpha)]; % move from i-1 to i
                nn = R*(nn + cross(R*pStar, f)) + cross(pStar + r, Fm(:, j)) + Nm(j);
                f = R*f + Fm(:, j);
                tau(j) = nn'*(R*z0);
            end
        end
    end
    
    methods (Access=private)
        
        function J = computeJacobian(obj, q)
            J = zeros(obj.N, obj.N); % initialize Jacobian
            U = eye(4, 4);
            for indx=6:-1:1
                T = obj.computeT(obj.DH(indx, :), q(indx));
                U = T*U;
                d = [-U(1,1)*U(2,4) + U(2,1)*U(1,4)
                    U(1,2)*U(2,4) + U(2,2)*U(1,4)
                    -U(1,3)*U(2,4) + U(2,3)*U(1,4)];
                delta = U(3,1:3)';  % nz oz az
                J(:,indx) = [d; delta];
            end
        end
        
        function diffMotionMatrix = computeError(obj, q, T)
            y = obj.solveFK(q);
            C = y\T;
            translMatrix = C(1:3, 4);
            rotMatrix = C(1:3, 1:3);
            R = rotMatrix - eye(3, 3);
            v = 0.5*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
            diffMotionMatrix = [translMatrix; v];
        end
        
        function status = isSolutionFound(obj, W, err)
            % check if the error is in the tolerance range
            status = obj.calculateNormError(W, err) < obj.Tolerance;
        end
        
    end
    
    methods (Static)
        
        function T = computeT(linkDH, q)
            T = [cos(q)     -sin(q)*cos(linkDH.alpha)   sin(q)*sin(linkDH.alpha)    linkDH.a*cos(q)
                sin(q)      cos(q)*cos(linkDH.alpha)    -cos(q)*sin(linkDH.alpha)   linkDH.a*sin(q)
                0           sin(linkDH.alpha)           cos(linkDH.alpha)           linkDH.d
                0           0                           0                           1];
        end
        
        function normError = calculateNormError(W, err)
            normError = norm(W*err);
        end
        
        %         function returnStatus(status, q, iteration)
        %             coder.extrinsic('warning');
        %             switch status
        %                 case int8(0)
        %                     string = fprintf('%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f', q(1), q(2), q(3), q(4), q(5), q(6));
        %                     message = 'No solution in the tolerance range could be found! Last iteration returned: q = [%s]';
        %                     warning(message, string);
        %                     %warning(message, strjoin(strtrim(cellstr(num2str(q(:)))), ', '));
        %                 case int8(1)
        %                     %fprintf('Solution found at %d iteration: \tq = [%s]\n', iteration, strjoin(strtrim(cellstr(num2str(q(:)))), ', '));
        %                     %string = fprintf('%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f', q(1), q(2), q(3), q(4), q(5), q(6));
        %                     %fprintf('Solution found at %d iteration: \tq = [%s]\n', int16(iteration), string);
        %                     %fprintf('Solution found at %d iteration: \tq = [%s]\n', iteration, strjoin(strtrim(cellstr(num2str(q(:)))), ', '));
        %                 case int8(2)
        %                     %message = 'All solutions have been rejected! No valid solution could be found.';
        %                     %warning(message);
        %             end
        %         end
    end
    
end

