function optimizerFPBI = prepareFPBI(paretoObj, optimizeConstraints, costExpressions, agent, extremePoints)
%PREPAREFPBI Summary of this function goes here
%   Detailed explanation goes here
persistent l f vectorStartingPoint

if agent.config.debugMode
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
else
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
end

normedEP = ParetoController.ParetoNormalization(extremePoints, paretoObj);

% if the default config for the focus point is used it has to be translated to a vector
if length(paretoObj.config.focusPoint) == 1
    focusPoint = paretoObj.config.focusPoint.*ones(1,numel(paretoObj.costFunctions));
else
    focusPoint =  paretoObj.config.focusPoint;
end

% find the indices of the farthest extreme points
largestDistanceIdc = extremePointsWithLargestDistance(normedEP);
midPoint = mean(normedEP(largestDistanceIdc,:));

focusVector = focusPoint(paretoObj.status.conflictingObj) - midPoint;
focusVector = focusVector/norm(focusVector);
paretoObj.status.focusVector = focusVector;

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

% generate normalized cost expressions
f = [costExpressions{:}];
f(conflictingObj) = ParetoController.ParetoNormalization([costExpressions{conflictingObj}],paretoObj);

% starting points as input
if isempty(vectorStartingPoint)
    vectorStartingPoint = sdpvar(1,numel(paretoObj.costFunctions));
end

FPBIConstraints = (vectorStartingPoint(conflictingObj) + l*focusVector >= f(conflictingObj)):'FPBI Constraint';
FPBISymbols = [vectorStartingPoint(conflictingObj)];

if isempty(paretoObj.status.redundantObj) && isempty(paretoObj.status.independantObj) && isempty(paretoObj.config.ignoreInPareto)
    optimizerFPBI = optimizer([optimizeConstraints; FPBIConstraints], -l, yalmipOptions, FPBISymbols, output);
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
    
    optimizerFPBI = optimizer([optimizeConstraints; FPBIConstraints], -l + addedObj, ...
        yalmipOptions, FPBISymbols, output);
end

end

%%
function idc = extremePointsWithLargestDistance(normedEP)
    combinations = nchoosek(1:size(normedEP,1),2);
    dist = nan(size(combinations,1),1);
    for i = 1:size(combinations,1)
        dist(i,1) = norm([1 -1]*normedEP(combinations(i,:),:));
    end
    idc = combinations(dist == max(dist),:);
end

