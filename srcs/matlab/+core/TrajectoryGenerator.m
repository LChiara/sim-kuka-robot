classdef TrajectoryGenerator < matlab.System & matlab.system.mixin.Propagates
    % System object to generate a trajectory for defined wa ypoints.
    %
    % A cubic polynomial or quintic polynomial, basing on the user input,
    % are used to generate the trajectory between specified waypoints.
    
    properties(Nontunable)
        %Method
        Method = 'Cubic Polynomial';
        %Waypoints
        Waypoints = zeros(3,10);
        %Time Points
        Timepoints = [0 1 2 3 4 5 6 7 8 9];
        %Velocity
        Velocities = zeros(3,10);
        %Acceleration
        Accelerations = zeros(3,10);        
    end
    
    properties(Constant, Hidden)
        MethodSet = matlab.system.StringSet({...
            'Cubic Polynomial', ...
            'Quintic Polynomial', ...
            })
    end
    
    properties(DiscreteState)
    end
    
    % Pre-computed constants
    properties(Access = private)
        NCoeff % computed basing on the Polynomial
    end
    
    methods(Access = protected)
        
        function validatePropertiesImpl(obj)
            % Establish dimensions
            n = size(obj.Waypoints, 1); %waypoint dimension -> n=3
            p = size(obj.Waypoints, 2); %number of defined waypoints
            
            % attribute validation
            validateattributes(obj.Waypoints, {'numeric'}, {'2d', 'nonempty', 'real', 'finite'}, 'TrajectoryGenerator', 'waypoints');
            validateattributes(obj.Timepoints, {'numeric'}, {'nonempty', 'vector', 'real', 'finite', 'increasing', 'nonnegative'}, 'TrajectoryGenerator', 'timepoints');
            
            coder.internal.errorIf(length(obj.Timepoints) ~= p, ...
                'core:trajectorygenerator:TimepointsDimensionMismatch');
            coder.internal.errorIf(size(obj.Velocities, 1) ~= n || size(obj.Velocities, 2) ~= p, ...
                'trajectoryPlanning:VelocitiesDimensionMismatch');
            if strcmp(obj.Method, 'Quintic Polynomial')
                coder.internal.errorIf(size(obj.Accelerations, 1) ~= n || size(obj.Accelerations, 2) ~= p, ...
                    'trajectoryPlanning:AccelerationsDimensionMismatch');
            end
        end
        
        function setupImpl(obj)
            if strcmp(obj.Method, 'Quintic Polynomial')
                obj.NCoeff = 6; % number of coefficients for quintic polynomial
            else
                obj.NCoeff = 4; % number of coefficients for cubic polynomial
            end
        end
        
        function [q_out, qd_out, qdd_out] = stepImpl(obj, t)
            
            % Caching the properties as local variables, when accessed
            % multiple times. Iterative calculations using cached local
            % variables tun faster than calculations accessing the
            % properties of an object.
            waypoints       = obj.Waypoints;
            velocities      = obj.Velocities;
            accelerations   = obj.Accelerations;
            timepoints      = obj.Timepoints;
            
            n = size(waypoints, 1);
            p = size(waypoints, 2);
            
            index = 1;
            % get index corresponding to time interval
            for i=1:p-1
                if t >= obj.Timepoints(i) && t < obj.Timepoints(i+1)
                    index = i;
                    break;
                end
            end
            
            pos_out = zeros(n, 1);
            v_out = zeros(n, 1);
            a_out = zeros(n, 1);
            for j = 1:n %calculate polynomial for each dimension
                if strcmp(obj.Method, 'Cubic Polynomial')
                    [q, qd, qdd] = trajectory.cubicPolynomial( ...
                        waypoints(j, index), ...
                        waypoints(j, index+1), ...
                        velocities(j, index), ...
                        velocities(j, index+1), ...
                        timepoints(index), ...
                        timepoints(index+1), ...
                        t);
                else
                    [q, qd, qdd] = trajectory.quinticPolynomial( ...
                        waypoints(j, index), ...
                        waypoints(j, index+1), ...
                        velocities(j, index), ...
                        velocities(j, index+1), ...
                        accelerations(j, index), ...
                        accelerations(j, index+1), ...
                        timepoints(index), ...
                        timepoints(index+1), ...
                        t);
                end
                pos_out(j, 1) = q;
                v_out(j, 1) = qd;
                a_out(j, 1) = qdd;
            end
            
            q_out = pos_out;
            qd_out = v_out;
            qdd_out = a_out;
        end
        
        function resetImpl(~)
            
        end
            
        function num = getNumInputsImpl(~)
            %getNumInputsImpl Define total number of inputs for system
            num = 1;
        end
        
        function num = getNumOutputsImpl(~)
            %getNumOutputsImpl Define total number of outputs for system
            num = 3;
        end
        
        function [out,out2,out3] = getOutputDataTypeImpl(~)
            %getOutputDataTypeImpl Return data type for each output port            
            out = 'double';
            out2 = 'double';
            out3 = 'double';
        end
        
        function [out,out2,out3] = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            n = size(obj.Waypoints, 1);
            out = [n 1];
            out2 = [n 1];
            out3 = [n 1];
        end
        
        function [out,out2,out3] = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
        end
        
        function [out, out2, out3] = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port if complex
            out = false;
            out2 = false;
            out3 = false;
        end
        
        function icon = getIconImpl(obj)
            %getIconImpl Define icon for System block
            filepath = fullfile('srcs', 'simulink', 'blockicons', 'polynomial-icon-big.jpg');
            icon = matlab.system.display.Icon(filepath);
        end
    end
end
