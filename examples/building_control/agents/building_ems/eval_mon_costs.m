function [value] = eval_mon_costs(agent, ~, predict, ~)
    if predict
         value = NaN( length(agent.status.uPred) );
    else
        % slack variable is only relevant for horizon
        if agent.status.k > 0
            init = agent.history.evalValues.monetary_costs(:, end);
        else
            init = 0;
        end
        
        value = init + agent.status.costsPred.monetary_costs(1);
    end
end