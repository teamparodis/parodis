function [PeffectiveLoad] = eval_PeffectiveLoad(agent, simulation, predict, s)
%EVAL_PCHARGE Returns Pcharge

if predict
    PeffectiveLoad = agent.status.dPred{s}(1,:)+agent.status.dPred{s}(2,:);
else
    PeffectiveLoad = agent.history.d(1, end) + agent.history.d(2, end);
end


