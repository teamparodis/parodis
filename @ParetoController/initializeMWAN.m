function [extremePoints, inputsEP, slacksEP, parametersEP] = initializeMWAN(paretoObj, optimizeConstraints, costExpressions, agent)
%MWA: Fast approximation for the EP of a MOO.
%    Algorithm for finding extreme points of the pareto front. Set one
%    objective weight to 1 and the rest to 1e-5. This is a good and fast
%    approach for the extreme points.

n = numel(paretoObj.status.conflictingObj) - numel(paretoObj.config.ignoreInPareto);
UPNP = zeros(n);
extremePoints = zeros(n);
inputsEP = cell(n,1);
slacksEP = cell(n,1);

weights = eye(n);

if isequal(paretoObj.config.normalization, 'dynamic') || length(paretoObj.config.fixedNormValues) ~= length(points)
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        zeros(1,numel(paretoObj.status.conflictingObj)), ones(1,numel(paretoObj.status.conflictingObj)));
    
elseif isequal(paretoObj.config.normalization, 'fixed')
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        zeros(1,numel(paretoObj.status.conflictingObj)), paretoObj.config.fixedNormValues);
    
else
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        zeros(1,numel(paretoObj.status.conflictingObj)), ones(1,numel(paretoObj.status.conflictingObj)));
    warning("Normalization method not recognized, will apply dynamic normalization. Try 'dynamic' or 'fixed'.")
end
% calculate the minimal value of each objective to get the utopia point and
% approximate the nadir point with the other objective values
for objective = 1:n    
    optOut = optimizer(weights(objective,:));
    
    UPNP(objective,:) = paretoObj.calculateUnnormedObjectiveValues(optOut, agent);
end

paretoObj.status.utopia = min(UPNP);
paretoObj.status.nadir  = max(UPNP);

weights(weights == 0) = paretoObj.config.MWAweights;

if isequal(paretoObj.config.normalization, 'dynamic') || length(paretoObj.config.fixedNormValues) ~= length(points)
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        paretoObj.status.utopia, paretoObj.status.nadir);
    
elseif isequal(paretoObj.config.normalization, 'fixed')
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        paretoObj.status.utopia, paretoObj.config.fixedNormValues);
    
else
    optimizer = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
        paretoObj.status.utopia, paretoObj.status.nadir);
    warning("Normalization method not recognized, will apply dynamic normalization. Try 'dynamic' or 'fixed'.")
end

% approximate the extreme points by applying a strong weighting to one and
% a weak one to the other normalized cost functions
for objective = 1:n 
    optOut = optimizer(weights(objective,:));
    
    [extremePoints(objective,:), inputsEP{objective,1}, slacksEP{objective,1}] = paretoObj.calculateUnnormedObjectiveValues(optOut, agent);
end

paretoObj.status.nadir = max(extremePoints);
parametersEP = weights;

end