classdef LQRCostFunction < CostFunction
    %LQRCOSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Q
        R
        Q_f
        Q_rep
        R_rep
        ref
        N_pred
    end
    
    methods
        function obj = LQRCostFunction(N_pred, Q, R, ref)
            Q_f = Q;
            if nargin < 4
                ref = repmat(zeros(length(Q), 1), 1, N_pred+1);
            end
            
            if isnumeric(ref) && all(size(ref) == [length(Q), 1])
                ref = repmat(ref, 1, N_pred+1);
            end
            
            obj.Q = Q;
            obj.R = R;
            obj.Q_f = Q_f;
            Q_rep = kron(eye(N_pred), Q);
            
            obj.Q_rep = [Q_rep zeros(length(Q_rep), length(Q));
                         zeros(length(Q), length(Q_rep)) Q_f];
            obj.R_rep = kron(eye(N_pred), R);
            obj.ref = ref;
            obj.N_pred = N_pred;
        end
        
        function expr = buildExpression(obj, x, u, d, params, Ns, slacks, T_s)
            expr = 0;
            for s=1:Ns
                expr = expr + obj.buildExpressionSingleScen(x{s}, u, d{s}, extractScenario(params, s), slacks, T_s, s);
            end
        end
        
        function [exprSingle] = buildExpressionSingleScen(obj, x, u, d, params, slacks, T_s, s)
            x_flat = x(:);
            u_flat = u(:);
            x_ref = obj.ref;
            
            % if reference signal is a char/string, then it's a parameter
            if ischar(obj.ref) || isstring(obj.ref)
                % check if parameter is scenario dependent
                if iscell(params.(x_ref))
                    x_ref = params.(x_ref){s};
                else
                    x_ref = params.(x_ref);
                end
                
                % if reference signal has only one column, it's a reference point
                if size(x_ref, 2) == 1
                    x_ref = repmat(x_ref, 1, obj.N_pred+1);
                end
            end   
            x_ref = x_ref(:);
            
            exprSingle = (x_flat-x_ref)' * obj.Q_rep * (x_flat-x_ref) + u_flat' * obj.R_rep * u_flat;
        end
        
        function [horizon] = evaluateHorizon(obj, x, u, d, params, slacks, T_s)
            x_ref = obj.ref;
            % if reference signal is a char/string, then it's a parameter
            if ischar(obj.ref) || isstring(obj.ref)
                % check if parameter is scenario dependent
                if iscell(params.(x_ref))
                    x_ref = params.(x_ref){1};
                else
                    x_ref = params.(x_ref);
                end
                
                % if reference signal has only one column, it's a reference point
                if size(x_ref, 2) == 1
                    x_ref = repmat(x_ref, 1, obj.N_pred+1);
                end
            end   
            
            for i=1:size(u, 2)
                horizon(i) = (x{1}(:, i) - x_ref(:, i))' * obj.Q * (x{1}(:, i) - x_ref(:, i)) + u(:, i)' * obj.R * u(:, i);
            end
        end
    end
end

