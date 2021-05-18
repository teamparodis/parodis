classdef CostFunction < handle
    % Abstract super class for implementation of cost functions
    
    methods (Abstract,Static)
        
        % Function returning an expression for the cost function
        [expr] = buildExpression(x, u, d, params, Ns, slacks, T_s)
    end
    
    methods (Static)
        % Function for introducing custom slacks and corresponding constraints
        function [slacks] = getSlacks(model, agent, params)
            slacks = struct;
        end
        
        function [constraints] = getConstraints(model, agent, slacks, params)
            constraints = [];
        end
        
        % Optional function for expression for a single scenario
        function [exprSingle] = buildExpressionSingleScen(x, u, d, params, slacks, T_s)
            exprSingle = [];
        end
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            % horizon = evaluateHorizon(x, u, d, params, slacks, T_s)    
            %   optional function that calculates l_i for each point in the
            %   horizon and returns [l_i(0|k) ... l_i(N_pred-1|k)]
            %
            % x         cell array with scenarios of xPred
            % u         uPred
            % d         cell array with scenarios of dPred
            % params    parameters
            % slacks    slacks
            % T_s       horizon time vector
            
            horizon = NaN(1, size(u, 2));
        end
    end
end