function [handle] = eval_const(constant)
% EVAL_CONST Helper function which returns an eval function which will simply return the given constant
%   Useful for adding constant lines, e.g. for constraints, into plots

handle = @(agent, simulation, predict, scenario)( eval_const_evaluation(constant, agent, predict) );
end

function [value] = eval_const_evaluation(constant, agent, predict)
%EVAL_CONSTANT_EVALUATION Eval function which simply repacks the given constant
% If predict = true it repeats the constant over the given horizon and
% wraps it in a scenario cell array
% Otherwise simply returns the constant

    if predict
        N_S = length(agent.model.x);
        value = repmat(constant, 1, length(agent.config.T_s));
    else
        value = constant;
    end
end

