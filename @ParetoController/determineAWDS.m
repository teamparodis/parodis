function [inputs, slacks, front, parameters] = determineAWDS(paretoObj, agent, optimizer, extremePoints, preselectedWeights)
% AWDS: Adaptive Weight Determination Scheme. Function for evaluating
%   an evenly distributed Pareto Front. Geometrical approach, that
%   interprets the weights of the weighted sum method as plane parameters.
%   A new Pareto-optimal point can thus be found between parents by using
%   them to calculate the planes parameters.
%
% Input:
%   extremePoints: Extreme Points found by an initialization algorithm,
%       e.g. 'lexicographic'. Points minimizing exactly one cost function.
%
% Output:
%   front: Pareto-optimal points.
%   inputs: All possible inputs u determined by evaluating different
%       Pareto-optimal points.
%   evaluatedWeights: Evaluated weights for logging purposes.

if nargin == 4 || isempty(preselectedWeights)
    paretoObj.paretoCurrentStep = 0;
    front = [];
    paretoObj.evaluatedPoints = extremePoints;
    numEP = size(extremePoints,1);
    
    % if the minimal distance to all other Pareto-optimal points is not set it
    % is set to a default values of 1/5 of the minimal distance ratio
    if isempty(paretoObj.config.distance2AllMin)
        paretoObj.config.distance2AllMin = paretoObj.config.drMin/5;
    end
    
    % if the number of points is less than the number of conflicting objectives
    % the AWDS cannot be applied
    if numEP < numel(paretoObj.status.conflictingObj)
        warning("Not enough points found for AWDS! :(")
        inputs = [];
        weights = [];
        slacks = [];
        return
    end
    
    % get all possible recombinations, if more than n EP candidates are given
    recombinations = nchoosek( 1:(numEP), numel(paretoObj.status.conflictingObj) );
    inputs = [];
    weights = [];
    slacks = [];
    
    % do AWDS for all combinations of candidates
    for ii = 1:size(recombinations,1)
        [frontTemp, inputsTemp, slacksTemp, evaluatedWeightsTemp] = evaluateParetoFront(paretoObj, agent, optimizer, extremePoints(recombinations(ii,:),:), Inf);
        front = [front; frontTemp];
        inputs = [inputs; inputsTemp];
        weights = [weights; evaluatedWeightsTemp];
        slacks = [slacks, slacksTemp];
    end
    
    % eliminate all weakly Pareto-optimal points
    filteredFront = ParetoController.paretoFilter(paretoObj, front, 1:numEP);
    front = front(filteredFront,:);
    parameters = weights(filteredFront,:);
    inputs     = inputs(filteredFront);
    slacks     = slacks(filteredFront);
    
elseif nargin == 5
    optOut = optimizer(preselectedWeights);
    
    [front, inputs, slacks] = paretoObj.calculateUnnormedObjectiveValues(optOut, agent);
    parameters = preselectedWeights;
else
    error("Input error in determineAWDS, not enough inputs.")
end

paretoObj.paretoCurrentStep = [];

end

%%
function [points, inputs, slacks, weights] = evaluateParetoFront(paretoObj, agent, optimizer, parents, drPrev)
% Recursive function for evaluating the Pareto front. Takes n parents
% that are used to define a plane. The plane's parameters are used as
% weights for a new point.

nConflict = numel(paretoObj.status.conflictingObj);
paretoObj.paretoCurrentStep = paretoObj.paretoCurrentStep + 1;
agent.simulation.updateProgress();

% calculate new weight as the plane parameters of the plane spanned by
% the parents
newWeight = hyperplane(parents, paretoObj);

% negative weights cannot be applied, if the ratio of 2 weights is larger
% 1e5 the solver approximates the smaller with 0 resulting in weakly
% Pareto-optimal solutions.
if any(newWeight <= 0) || min(newWeight)/max(newWeight) < paretoObj.config.minRatioWeights
    points = [];
    weights = [];
    inputs = [];
    slacks = [];
    return
end

% optimize model for finding ne optimal u
[optOut, feasibilityCode] = optimizer(newWeight);

% skip the solution if the problem was infeasible
if feasibilityCode ~= 0
    msg = "Yalmip error in Simulation step " + agent.sim.status.k + " in Pareto step " + paretoObj.paretoCurrentStep + ": " + (feasibilityCode);
    if paretoObj.config.printSolverStatus
        agent.log(msg); %possible: yalmiperror
    else
        warning(msg);
    end
    points = [];
    weights = [];
    inputs = [];
    slacks = [];
    return
end

[J_opt, u, sl] = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);

% calculate the minimal distance to all other Pareto-optimal points
distance2All = minDistance2all(J_opt, paretoObj);

if distance2All <= paretoObj.config.distance2AllMin || isinf(distance2All)
    % Check if there is a found solution similar to the new one, since
    % this would only result in more similar solutions.
    points = [];
    weights = [];
    inputs = [];
    slacks = [];
    return
end
paretoObj.evaluatedPoints(end+1,:) = J_opt;

% update the nadir point if the new point is bigger in one
% objective
if any(paretoObj.status.nadir < J_opt)
    paretoObj.status.nadir = max([paretoObj.status.nadir; J_opt]);
end

inputs = {u};
slacks = [sl];
points = J_opt;
weights = newWeight;

% get the candidates for new parents
candidates = unique(parents,'rows','stable');

if size(candidates,1) < nConflict-1
    points = [];
    weights = [];
    inputs = [];
    slacks = [];
    return
end

recombinations = nchoosek( 1:(size(candidates,1)), nConflict - 1);

for jj = 1:size(recombinations,1)
    newParents = [candidates(recombinations(jj,:),:); J_opt];
    prevPoints = parents;
    
    if isempty(prevPoints)
        continue
    end
    
    dr = distance_ratio(prevPoints, J_opt, paretoObj);
    
    if dr <= paretoObj.config.drMin || ((dr/drPrev >= 1-1e-4) && (dr/drPrev <= 1+1e-4))
        % distance ratio is used to predict whether new solutions will be
        % found. Also prevents agglomeration of new solutions.
        continue
    end
    
    [newJ_opt, u, newSlacks, JOptWeight] = evaluateParetoFront(paretoObj, agent, optimizer, newParents, dr);
    
    inputs = [inputs; u];
    slacks = [slacks; newSlacks];
    points = [points; newJ_opt];
    weights = [weights; JOptWeight];
end
end

%%
function [planeParameters] = hyperplane(hyperpoints, paretoObj)
% calculates hyperplane parameters a1,a2,a3...  for hyperplane a1*f1+a2*f2+a3*f3+...=1

normedHyperpoints = ParetoController.ParetoNormalization(hyperpoints, paretoObj);
planeParameters = (normedHyperpoints\ones(size(normedHyperpoints, 1), 1))';

% plane parameters should always all be positive, all negative case
% changes only offset on y-axis, and since this is not needed, we turn
% negative into positive
if (all(planeParameters <= 0))
    planeParameters = -planeParameters;
end

if any(planeParameters < 0)
    proxyPP1 = planeParameters;
    proxyPP2 = -planeParameters;
    
    proxyPP1(proxyPP1 < 0) = 1e-5;
    proxyPP2(proxyPP2 < 0) = 1e-5;
    
    proxyPP1 = proxyPP1/norm(proxyPP1);
    proxyPP2 = proxyPP2/norm(proxyPP2);
    
    if abs(planeParameters*proxyPP1') >= abs(planeParameters*proxyPP2') && planeParameters*proxyPP1' >= 0.95
        planeParameters = proxyPP1;
    elseif abs(planeParameters*proxyPP2') > abs(planeParameters*proxyPP1') && planeParameters*proxyPP2' >= 0.95
        planeParameters = proxyPP2;
    end
end
planeParameters = planeParameters/sum(abs(planeParameters));
end

%%
function dr = distance_ratio(previousPoints, newPoint, paretoObj)
% calculates the distance ratio between previously found solutions and the
% new solution. in the case of n=2 dimensions, distance ratio is equal to
% the euclidian distance in normalized (decision) space.
numParents = numel(paretoObj.status.conflictingObj);
r = paretoObj.status.nadir - paretoObj.status.utopia;
dr = 0;

for p=1:numParents-1
    dr = dr + norm( (newPoint - previousPoints(p, :))./ r );
end

dr = dr/(numParents-1);

end

%%
function distance2All = minDistance2all(newPoint, paretoObj)
% Calculates the minimal distance to any previously found solution.
% Prevents accumulation of similar solutions.
checkPoints = ParetoController.ParetoNormalization(paretoObj.evaluatedPoints, paretoObj);
distance2All = min(vecnorm(checkPoints - ParetoController.ParetoNormalization(newPoint, paretoObj),2,2));

end