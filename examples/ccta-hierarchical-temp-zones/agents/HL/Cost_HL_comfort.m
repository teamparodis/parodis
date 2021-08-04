classdef Cost_HL_comfort < CostFunction
    %COMFORTCOSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        
        
        function [slacks] = getSlacks(model, agent, params) 
            slacks = struct;
            slacks.theta_aux = sdpvar(length(agent.config.T_s)+1, 1);        
        end
        
        function [constraints] = getConstraints(model, agent, slacks, params)
            constraints = (slacks.theta_aux == model.x{1}(2, :)' - params.theta_ref{1}):'auxiliary slack variable for (theta - theta_ref)';       
        end
               
        function expr = buildExpression(x, u, d, params, Ns, slacks, T_s)
            expr = 0;
            for s=1:Ns
                expr = expr + Cost_HL_comfort.buildExpressionSingleScen(x{s}, u, d{s}, extractScenario(params, s), slacks, T_s);
            end
        end
        
        function [exprSingle] = buildExpressionSingleScen(x, u, d, params, slacks, T_s)
            T_s_extended = [T_s mean(T_s)];
            exprSingle =  numel(T_s_extended) * (T_s_extended'./sum(T_s_extended)  .* slacks.theta_aux)' * slacks.theta_aux;
        end
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            theta = x{1}(2, 1:end-1);
            %horizon = slacks.theta_ref;
            horizon = numel(T_s) * T_s/sum(T_s).* (theta - params.theta_ref{1}).^2;
        end
    end

end


