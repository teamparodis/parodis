function [costs] = eval_electricity_costs(agent, simulation, predict, scenario)
%EVAL_ELECTRICITY_COSTS Returns the electricity costs

price = agent.status.paramValues.price_pred{1};
if issymmetric(price)
    price = diag(price)';
end

if predict
    costs = (price * 100);
else
    costs = (price(1) * 100);
end


