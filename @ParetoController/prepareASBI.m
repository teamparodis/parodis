function optimizerASBI = prepareASBI(paretoObj, optimizeConstraints, costExpressions, agent, ~)
% Generate the optimizer used in the ASBI method
persistent l f vectorStartingPoint searchVector normalVector

if agent.config.debugMode
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
else
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
end

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

conflictingObj = paretoObj.status.conflictingObj;
nCostFunction = numel(paretoObj.costFunctions);

% generate normalized cost expressions
f = [costExpressions{:}];
f(conflictingObj) = ParetoController.ParetoNormalization([costExpressions{conflictingObj}],paretoObj);

% starting point from where the search vector is starting
if isempty(vectorStartingPoint)
    vectorStartingPoint = sdpvar(1,nCostFunction);
end

% normal vector = search direction
if isempty(normalVector)
    normalVector = sdpvar(1, nCostFunction);
end

% search vector is parallel to the normal vector
if isempty(searchVector)
    searchVector = sdpvar(1, nCostFunction);
end

ASBIConstraints = [(vectorStartingPoint(conflictingObj) + searchVector(conflictingObj) >= f(conflictingObj)):'ASBI Constraint', ...
                    (searchVector(conflictingObj) == l * normalVector(conflictingObj)):'search vector parallel to normal vector'];
ASBISymbols = [vectorStartingPoint(conflictingObj) normalVector(conflictingObj)];

noRedundancy = isempty(paretoObj.status.redundantObj) && isempty(paretoObj.status.independantObj);

if noRedundancy && isempty(paretoObj.config.ignoreInPareto)  
    optimizerASBI = optimizer([optimizeConstraints; ASBIConstraints], searchVector*...
        ones(nCostFunction,1), yalmipOptions, ASBISymbols, output);
    
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
    
    optimizerASBI = optimizer([optimizeConstraints; ASBIConstraints], sum(searchVector) + addedObj, ...
        yalmipOptions, ASBISymbols, output);
end

end