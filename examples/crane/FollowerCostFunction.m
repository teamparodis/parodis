classdef FollowerCostFunction < CostFunction
    %COMFORTCOSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function slacks = getSlacks(model, agent, params)
            slacks = struct;
            N_S = length(model.x);
            
            slacks.x_ref_aux = sdpvar(N_S, length(agent.config.T_s)+1);
        end
        
        function constraints = getConstraints(model, agent, slacks, params)
            constraints = [];
            N_S = length(model.x);
            
            for s=1:N_S
                constraints = [
                    constraints;
                    (slacks.x_ref_aux(s, :) == model.x{s}(1, :) - params.x_ref{s}):sprintf('auxiliary slack variable for (x - x_ref) s = %i', s);
                ];
            end
        end
        
        function expr = buildExpression(x, u, d, params, Ns, slacks, T_s)
            expr = 0;
            for s=1:Ns
                expr = expr + FollowerCostFunction.buildExpressionSingleScen(x{s}, u, d{s}, extractScenario(params, s), slacks, s);
            end
        end
        
        function [exprSingle] = buildExpressionSingleScen(x, u, d, params, slacks, s)
            exprSingle = slacks.x_ref_aux(s, :) * slacks.x_ref_aux(s, :)';
        end
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            x_pos = x{1}(2, 1:end-1);
            horizon = (x_pos - params.x_ref{1}(1:end-1)).^2;
        end
    end

end

