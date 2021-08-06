function [QiMultiplied] = eval_QiMultiplied(agent, simulation, predict, s)
%EVAL_QUIMULTIPLIED Returns Q_heat_1 * Q_cool_1
useState = 1; 
if predict
    QiMultiplied = agent.status.uPred(useState,:).*agent.status.uPred(useState+9,:);
else
    QiMultiplied = agent.history.u(useState, end) * agent.history.u(useState+9, end);
end