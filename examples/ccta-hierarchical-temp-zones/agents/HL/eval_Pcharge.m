function [Pcharge] = eval_Pcharge(agent, simulation, predict, s)
%EVAL_PCHARGE Returns Pcharge

if predict
    dSoC = agent.status.xPred{s}(1, 2:end) - agent.status.xPred{s}(1, 1:end-1);
    dT = agent.config.T_s / 60;
    Pcharge = dSoC ./ dT;
else
    dT = agent.config.T_s(1) / 60;
    Pcharge = (agent.history.x(1, end) - agent.history.x(1, end-1)) / dT;
end


