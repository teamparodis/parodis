function loadDataToStatus(app)
% fill the agents status for trajectory prediction
if app.useSim
    % check if inputs is a valid field of controller status and if all of them are empty
    if ~isfield(app.agentObj.controller.status,'inputs') || all(cellfun(@isempty, app.agentObj.controller.status.inputs))
        % if all are empty fill agent
        app.simObj.loadDataFromSimulation(app.simObj.simulationName, app.timeStep - 1, string(app.agentObj.name));
        app.agentObj.updateRng();
        app.agentObj.measureState( );
        app.agentObj.status.dPred = app.agentObj.getDisturbance( );
        app.agentObj.setParameterValues();
        app.agentObj.controller.status.inputs = cell(1,size(app.agentObj.history.pareto.paretoParameters{app.timeStep},1));
        if ~isempty(app.history.(app.AgentDropDown.Value).pareto.frontDeterminationScheme{app.timeStep}) && all(~isnan(app.history.ems.pareto.frontDeterminationScheme{app.timeStep}))
            app.agentObj.controller.config.frontDeterminationScheme = app.history.ems.pareto.frontDeterminationScheme{app.timeStep};
        end
    end
    if isempty(app.agentObj.controller.status.inputs{app.chosenIdx})
        % calculate the input if it wasn't already calculated
        app.agentObj.controller.status.inputs{app.chosenIdx} = app.agentObj.controller.getInput(app.agentObj.history.x(:,app.timeStep),...
            app.agentObj,app.agentObj.history.pareto.paretoParameters{app.timeStep}(app.chosenIdx,:));
    end
end
app.agentObj.status.uPred = app.agentObj.controller.status.inputs{app.chosenIdx}; % calculate input
end