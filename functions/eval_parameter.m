function [handle] = eval_parameter(param, index)
% EVAL_CONST Helper function which returns an eval function which will return the value of the given parameter
%   Useful for parameter values within the prediction horizon. Since parameters don't take any values outside the
%   horizon, the value stored in the eval history will be NaN
if nargin < 2
    index = 1;
end

handle = @(agent, simulation, predict, scenario)( eval_parameter_evaluation(param, index, agent, predict, scenario) );
end

function [value] = eval_parameter_evaluation(param, index, agent, predict, scenario)

N_pred = length(agent.config.T_s);
if predict
    value = agent.status.paramValues.(param){scenario}(index, :);
    if length(value) > N_pred
        value = value(1:N_pred);
    end
else
    value = NaN;
end

end

