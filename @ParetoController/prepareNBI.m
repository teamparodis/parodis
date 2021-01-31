function optimizerNBI = prepareNBI(paretoObj, optimizeConstraints, costExpressions, agent, extremePoints)
%Generate the optimizer used in the standard NBI method
persistent l f vectorStartingPoint

if agent.config.debugMode
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
else
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
end

normedEP = ParetoController.ParetoNormalization(extremePoints, paretoObj);
normalVector = -(normedEP(:, paretoObj.status.conflictingObj)\ones(numel(paretoObj.status.conflictingObj),1))';

options = [options, agent.config.solverOptions];
yalmipOptions = sdpsettings( options{:} );
output = {agent.model.u};
slackVariableNames = fieldnames(paretoObj.slackVariables);

for idx=1:length(slackVariableNames)
    output{end+1} = paretoObj.slackVariables.(slackVariableNames{idx});
end

if isempty(l)
    l = sdpvar(1);
end

% generate normalized cost expressions
f = ParetoController.ParetoNormalization([costExpressions{:}], paretoObj);

if isempty(vectorStartingPoint)
    vectorStartingPoint = sdpvar(1,numel(paretoObj.costFunctions));
end

conflictingObj = paretoObj.status.conflictingObj;
NBIConstraints = (vectorStartingPoint(conflictingObj) + l*normalVector >= f(conflictingObj)):'NBI Constraint';
NBISymbols = [vectorStartingPoint(conflictingObj)];

if isempty(paretoObj.status.redundantObj) && isempty(paretoObj.status.independantObj)
    optimizerNBI = optimizer([optimizeConstraints; NBIConstraints], -l, yalmipOptions, NBISymbols, output);
else
    addedObj = 0;
    if ~isempty(paretoObj.status.independantObj)
        addedObj = addedObj + paretoObj.config.indepWeight*...
            f(paretoObj.status.independantObj)*ones(numel(paretoObj.status.independantObj),1);
    end
    if ~isempty(paretoObj.status.redundantObj)
        addedObj = addedObj + paretoObj.config.redundantWeight*...
            f(paretoObj.status.redundantObj)*ones(numel(paretoObj.status.redundantObj),1);
    end
    
    optimizerNBI = optimizer([optimizeConstraints; NBIConstraints], -l + addedObj, ...
        yalmipOptions, NBISymbols, output);
end

end