function loadFullHistory(app)
%LOADFULLHISTORY Summary of this function goes here
%   Detailed explanation goes here

agentNames = fieldnames(app.simObj.agents);

for i = 1:length(agentNames)
    app.simObj.loadDataFromSimulation(app.simObj.simulationName, app.simObj.T_sim/...
        app.simObj.agents.(agentNames{i}).config.T_s(1), string(string(app.simObj.agents.(agentNames{i}).name)));
    app.history.(agentNames{i}) = app.simObj.agents.(agentNames{i}).history;
end

end

