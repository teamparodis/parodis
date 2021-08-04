function QheatFromHL = param_QheatFromHL(T, callingAgent, agents, numScenarios);
% QHEATFROMHL Source function to read predicted overall Qheat from HL

P_chp = agents.HigherLevel.previousStatus.uPred(2,:); 
Q_rad = agents.HigherLevel.previousStatus.uPred(3,:); 
c_chp = 0.677;

QheatFromHL = {Q_rad + P_chp/c_chp};

end