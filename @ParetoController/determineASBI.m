function [inputs, slacks, front, parameters] = determineASBI(paretoObj, agent, optimizer, extremePoints, preselectedParameters)
% ASBI: Adaptive Search Boundary Intersection. Function for evaluating an evenly
% distributed Pareto front. Recursive approach
%   
% Input:
%   paretoObj: The ParetoController object calling this function.
%   agent: Agent object that this timestep is evaluated for.
%   optimizer: YALMIP optimizer object for this determination method.
%   extremePoints: Extreme Points found by an initialization algorithm,
%       e.g. 'lexicographic'. Points minimizing exactly one cost function.
%   preselectedParameters: Given when the paretoParameters are already fixed. If given,
%       this function will only call the optimizer with these parameters.
%
% Output:
%   front: Pareto optimal points.
%   inputs: All possible inputs u determined by evaluating different Pareto-optimal points.
%   slacks: Slack variables for every Pareto optimal point.    
%   parameters: Evaluated paretoParameters for logging purposes.

if nargin == 4 || isempty(preselectedParameters)
    paretoObj.paretoCurrentStep = 0;
    front = [];
    paretoObj.evaluatedPoints = extremePoints;
    numEP = size(extremePoints,1);
    
    % if the minimal distance to all other Pareto-optimal points is not set it
    % is set to a default values of 1/5 of the minimal distance ratio
    if isempty(paretoObj.config.distance2AllMin)
        paretoObj.config.distance2AllMin = paretoObj.config.drMin/5;
    end
    
    % get the number of unique Pareto-optimal points
%     numPoints = size(unique(extremePoints,'rows'),1);
    
    % if the number of points is less than the number of conflicting objectives the ASBI
    % cannot be applied
    if numEP < numel(paretoObj.status.conflictingObj)
        warning("Not enough points found for AWDS! :(")
        inputs = [];
        paretoParameters = [];
        slacks = [];
        return
    end
    
    % get all possible recombinations, if more than n EP candidates are given
    recombinations = nchoosek( 1:(numEP), numel(paretoObj.status.conflictingObj) );
    inputs = [];
    paretoParameters = [];
    slacks = [];
    
    % do ASBI for all combinations of candidates
    for ii = 1:size(recombinations,1)
        [frontTemp, inputsTemp, slacksTemp, evaluatedWeightsTemp] = evaluateParetoFront(paretoObj, agent, optimizer, extremePoints(recombinations(ii,:),:), Inf);
        front = [front; frontTemp];
        inputs = [inputs; inputsTemp];
        paretoParameters = [paretoParameters; evaluatedWeightsTemp];
        slacks = [slacks, slacksTemp];
    end
    
    % eliminate all weakly Pareto-optimal points
    filteredFront = ParetoController.paretoFilter(paretoObj, front, 1:numEP);
    front = front(filteredFront,:);
    parameters = paretoParameters(filteredFront,:);
    inputs     = inputs(filteredFront);
    slacks     = slacks(filteredFront);
    
elseif nargin == 5
    optOut = optimizer(preselectedParameters);
    
    [front, inputs, slacks] = paretoObj.calculateUnnormedObjectiveValues(optOut, agent);
    parameters = preselectedParameters;
else
    error("Input error in determineAWDS, not enough inputs.")
end

paretoObj.paretoCurrentStep = [];

end

%%
function [points, inputs, slacks, paretoParameters] = evaluateParetoFront(paretoObj, agent, optimizer, parents, drPrev)
% Recursive function for evaluating the Pareto front. Takes n parents defining a
% hyperplane. The starting point is the midpoint of all n parents, the search direction is
% the normal vector of the hyperplane.

nConflict = numel(paretoObj.status.conflictingObj);
paretoObj.paretoCurrentStep = paretoObj.paretoCurrentStep + 1;
agent.simulation.updateProgress();

newParameters = computeParetoParameters(parents, paretoObj);

% optimize with new paretoParameters for finding the optimal u
[optOut, feasibilityCode] = optimizer(newParameters);

% skip the solution if the problem was infeasible
if feasibilityCode ~= 0
    msg = "Yalmip error in Simulation step " + agent.simulation.k_sim + " in Pareto step " + paretoObj.paretoCurrentStep + ": " + (feasibilityCode);
    if paretoObj.config.printSolverStatus
        agent.log(msg); %possible: yalmiperror
    else
        warning(msg);
    end
    points = [];
    paretoParameters = [];
    inputs = [];
    slacks = [];
    return
end

[J_opt, u, sl] = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);

% calculate the minimal distance to all other Pareto-optimal points
distance2All = minDistance2all(J_opt, paretoObj);

if distance2All <= paretoObj.config.distance2AllMin || isinf(distance2All)
    % Check if there is a solution similar to the new one, since this would only result in
    % more similar solutions.
    points = [];
    paretoParameters = [];
    inputs = [];
    slacks = [];
    return
end
paretoObj.evaluatedPoints(end+1,:) = J_opt;

% update the nadir point if the new point is bigger in one objective
if any(paretoObj.status.nadir < J_opt)
    paretoObj.status.nadir = max([paretoObj.status.nadir; J_opt]);
end

inputs = {u};
slacks = [sl];
points = J_opt;
paretoParameters = newParameters;

% get the candidates for new parents
candidates = unique(parents,'rows','stable');

if size(candidates,1) < nConflict-1
    points = [];
    paretoParameters = [];
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
    
    if dr <= paretoObj.config.drMin
        % Distance ratio is used to predict whether new solutions will be found
        continue
    end
    
    [newJ_opt, u, newSlacks, JOptParameters] = evaluateParetoFront(paretoObj, agent, optimizer, newParents, dr);
    
    inputs = [inputs; u];
    slacks = [slacks; newSlacks];
    points = [points; newJ_opt];
    paretoParameters = [paretoParameters; JOptParameters];
end
end

%%
function [paretoParameters] = computeParetoParameters(hyperpoints, paretoObj)

normedHyperpoints = ParetoController.ParetoNormalization(hyperpoints, paretoObj);
midpoint = mean(normedHyperpoints);
searchDirection = null(normedHyperpoints(2:end,:)-normedHyperpoints(1,:))';

paretoParameters = [midpoint, searchDirection];
end

%%
function dr = distance_ratio(previousPoints, newPoint, paretoObj)
% calculates the distance ratio between previously found solutions and the
% new solution. in the case of n = 2 dimensions, distance ratio is equal to
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