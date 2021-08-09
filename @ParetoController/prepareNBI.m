function optimizerNBI = prepareNBI(paretoObj, optimizeConstraints, costExpressions, agent, extremePoints)
%Generate the optimizer used in the standard NBI method
persistent l f vectorStartingPoint

normedEP = ParetoController.ParetoNormalization(extremePoints, paretoObj);
normalVector = -(normedEP(:, paretoObj.status.conflictingObj)\ones(numel(paretoObj.status.conflictingObj),1))';

output = {agent.model.u};
slackVariableNames = fieldnames(paretoObj.slackVariables);

for idx=1:length(slackVariableNames)
    output{end+1} = paretoObj.slackVariables.(slackVariableNames{idx});
end

if isempty(l)
    l = sdpvar(1);
end

conflictingObj = paretoObj.status.conflictingObj;

% generate normalized cost expressions
f = [costExpressions{:}];
f(conflictingObj) = ParetoController.ParetoNormalization([costExpressions{conflictingObj}],paretoObj);

if isempty(vectorStartingPoint)
    vectorStartingPoint = sdpvar(1,numel(paretoObj.costFunctions));
end

NBIConstraints = (vectorStartingPoint(conflictingObj) + l*normalVector >= f(conflictingObj)):'NBI Constraint';
NBISymbols = [vectorStartingPoint(conflictingObj)];

if isempty(paretoObj.status.redundantObj) && isempty(paretoObj.status.independantObj) && isempty(paretoObj.config.ignoreInPareto)
    optimizerNBI = optimizer([optimizeConstraints; NBIConstraints], -l, agent.controller.yalmipOptions, NBISymbols, output);
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
    if ~isempty(paretoObj.config.ignoreInPareto)
        addedObj = addedObj + f(paretoObj.config.ignoreInPareto)*paretoObj.defaultWeights(paretoObj.config.ignoreInPareto)';
    end
    
    optimizerNBI = optimizer([optimizeConstraints; NBIConstraints], -l + addedObj, ...
        agent.controller.yalmipOptions, NBISymbols, output);
end

end