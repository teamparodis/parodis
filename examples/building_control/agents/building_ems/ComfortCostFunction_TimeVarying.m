classdef ComfortCostFunction_TimeVarying < CostFunction
    %COMFORTCOSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        
        
        function [slacks] = getSlacks(model, agent, params) 
            slacks = struct;
            
            % slack variable to reformulate (x-x_ref)² into x_aux²
            % this sparse reformulation is much faster than the explicit quadratic term
            slacks.theta_aux = sdpvar(length(agent.config.T_s)+1, 1);        
        end
        
        function [constraints] = getConstraints(model, agent, slacks, params)
            % constraint on x_aux == x-x_ref for reformulation
            constraints = (slacks.theta_aux == model.x{1}(2, :)' - params.theta_ref{1}):'auxiliary slack variable for (theta - theta_ref)';       
        end
               
        function expr = buildExpression(x, u, d, params, Ns, slacks, T_s)
            expr = 0;
            % Here, the cost expression in each scenario is the same, so simply sum up the costs over all scenarios
            for s=1:Ns
                expr = expr + ComfortCostFunction_TimeVarying.buildExpressionSingleScen(x{s}, u, d{s}, extractScenario(params, s), slacks, T_s);
            end
        end
        
        function [exprSingle] = buildExpressionSingleScen(x, u, d, params, slacks, T_s)
            % add the average sample as a weight for x(Npred), so the final costs
            T_s_extended = [T_s mean(T_s)];
            
            % since horizon has varying time steps T_s, costs over horizon are weighted accordingly
            exprSingle =  numel(T_s_extended) * (T_s_extended'./sum(T_s_extended)  .* slacks.theta_aux)' * slacks.theta_aux;
        end
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            % evaluateHorizon This function will return the costs as numerical values over the horizon
            % This is used for plotting and is stores in agent.status.costs.comfort_costs
            
            theta = x{1}(2, 1:end-1);
            %horizon = slacks.theta_ref;
            horizon = numel(T_s) * T_s/sum(T_s).* (theta - params.theta_ref{1}).^2;
        end
    end

end

