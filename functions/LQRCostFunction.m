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
            ref = obj.ref;
            if ischar(obj.ref)
                ref = params.(ref);
            end   
            ref = ref(:);
            
            exprSingle = (x_flat-ref)' * obj.Q_rep * (x_flat-ref) + u_flat' * obj.R_rep * u_flat;
        end
        
        function [horizon] = evaluateHorizon(obj, x, u, d, params, slacks, T_s)
            for i=1:size(u, 2)
                horizon(i) = x{1}(:, i)' * obj.Q * x{1}(:, i) + u(:, i)' * obj.R * u(:, i);
            end
        end
    end
end

