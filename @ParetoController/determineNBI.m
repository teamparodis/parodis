function [inputs, slacks, front, parameters] = determineNBI(paretoObj, agent, optimizer, extremePoints, preselectedStartingPoint)
%Normal Boundary Intersection
%   The normal boundary intersection projects points from the boundary
%   plane to the Pareto front to find Pareto-optimal points.

if nargin == 4 || isempty(preselectedStartingPoint)
    
n = numel(paretoObj.status.conflictingObj);
numEP = size(extremePoints,1);

front = extremePoints;
inputs = [];
startingPoints = [];

normedEP = ParetoController.ParetoNormalization(extremePoints,paretoObj);
N_k = normedEP(end,:)-normedEP(1:end-1,:);

% number of divisions between the extreme points
m = vecnorm(N_k,2,2)/paretoObj.config.distanceInBP;
delta_k = 1./(m-1);
delta_k(delta_k < 0 | delta_k > 1) = 1;
alpha_kj = {0:delta_k(1):1};

for k = 2:length(delta_k)
    alpha_kj = [alpha_kj;0:delta_k(k):1];
end

% the alphas are used as weights for a weighted sum of the extreme points to get the boundary plane points
alpha = getAlpha(alpha_kj);
planePoints = [normedEP(:,paretoObj.status.conflictingObj); alpha*normedEP(:,paretoObj.status.conflictingObj)];

paretoObj.paretoMaxStep = size(planePoints,1)-n;

for i = n+1:size(planePoints,1)
    paretoObj.paretoCurrentStep = i-n;
    agent.simulation.updateProgress();
    
    [optOut, feasibilityCode] = optimizer(planePoints(i,:));
    
    if feasibilityCode ~= 0
        continue
    end
    
    pos = i-n;
    [front(pos,:), inputs{pos,1}, slacks{pos,1}] = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);
    
    if isequal(front(pos,:),[0 0 0])
        continue
    end
    
    startingPoints = [startingPoints; planePoints(i,:)];
end

paretoObj.status.nadir = max(front);
filteredFront = ParetoController.paretoFilter(paretoObj, front, 1:numEP);
front = front(filteredFront,:);

filteredPIS = filteredFront - numEP;
filteredPIS(filteredPIS <= 0) = [];
parameters = startingPoints(filteredPIS,:);
inputs = inputs(filteredPIS);
slacks = slacks(filteredPIS);

elseif nargin == 5
    optOut = optimizer(preselectedStartingPoint);
    
    [front, inputs, slacks] = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);
else
    error("Input error in determineNBI, not enough inputs.")
end

paretoObj.paretoCurrentStep = [];
paretoObj.paretoMaxStep = [];

end

%%
function alpha = getAlpha(alpha_kj)
%getAlpha calculates the alphas for the combinations of extreme points to get the even distribution
%of the boundary plane

lenCell = cellfun(@(a) 1:length(a),alpha_kj,'Uni',0);
C = cell(1,numel(lenCell));
[C{:}] = ndgrid(lenCell{:});
C = cellfun(@(X) reshape(X,[],1),C,'UniformOutput',false);
alpha_recomb = horzcat(C{:});

alpha = [];
for j=1:size(alpha_recomb,1)
    alpha_sum = 0;
    for k=1:size(alpha_recomb,2)
        alpha_sum = alpha_sum + alpha_kj{k}(alpha_recomb(j,k));
    end
    if alpha_sum <= 1
        alpha(end+1,:) = [cellfun(@(a,j) a(j), alpha_kj, mat2cell(alpha_recomb(j,:),1,ones(size(alpha_recomb(j,:))))')', 1-alpha_sum]; %cellfun evaluates each alpha_kj at the j-th position
    end
end

alpha(any(alpha == 1,2),:) = []; % remove any combination resulting in the extreme points

end

