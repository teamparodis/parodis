function QcoolFromHL = param_QcoolFromHL(T, callingAgent, agents, numScenarios);
% QCOOLFROMHL Source function to read predicted Qcool from HL

QcoolFromHL = {agents.HigherLevel.previousStatus.uPred(4,:)};

end