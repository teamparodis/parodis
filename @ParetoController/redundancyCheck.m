function redundancyCheck(paretoObj, agent, optimizeConstraints, costExpressions)
% Check the cost functions for correlation
%   Correlation of cost function is checked by calculating representative
%   solutions with fixed weights. The bravais pearson correlation coefficient is applied.
%
%   Input:
%       paretoObj: Pareto class object
%       agent: Agent class objective

redundantObj = [];
independantObj = [];
pt = [];
w = [];

paretoObj.status.conflictingObj = ParetoController.paretoSetDiff(paretoObj.config.conflictingObj,...
    paretoObj.config.ignoreInPareto);

% set weights for optimization
n = numel(paretoObj.status.conflictingObj);
singleWeights = cell(1,n);
weightSteps = [0.3 1 10 50];

% generate all possible combinations of weights for weightsSteps
[singleWeights{:}] = deal(weightSteps);
weightsGrid = cell(1,n);
[weightsGrid{:}] = ndgrid(singleWeights{:});
weightsCell = cellfun(@(X) reshape(X,[],1),weightsGrid,'UniformOutput',false);
weights = horzcat(weightsCell{:});

% remove the weight combinations where all are equal and reintroduce one
weights(all(weights(:,1) == weights(:,2:end),2),:) = [];
weights = [weights; weightSteps(1)*ones(1,n)];

optimizerRedundancy = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent,...
                zeros(1,length(paretoObj.config.conflictingObj)),ones(1,length(paretoObj.config.conflictingObj)));

% solve weighted optimization
for j = 1:size(weights,1)
    
    [optOut, feasibilityCode] = optimizerRedundancy(weights(j,:));
    
    if feasibilityCode ~= 0
        continue
    end
    
   J_opt = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);
    
    pt(end+1,:) = J_opt;
    w(end+1,:) = weights(j,:);
end

% check for equal objectives
ptNorm = (pt-min(pt))./(max(pt)-min(pt));
ptCorrelating = ptNorm(all(w >= 1,2),:);
PCorrelating = corr(uniquetol(ptCorrelating,paretoObj.config.corrUniqueThreshold,'ByRows',true));

[equalRow, equalCol] = find(PCorrelating-eye(size(PCorrelating)) >= paretoObj.config.redundantCorrelation);
equalPairs = unique(sort([equalRow,equalCol],2),'rows');
[~, redundantObj, equalTuples] = eliminiateOverrepresentedObjectives(equalPairs, redundantObj);

for numTuples = 1:length(equalTuples)
    [~,keepIdx] = min(sum(PCorrelating(equalTuples{numTuples},:),2));
    redundantObj = [redundantObj, ParetoController.paretoSetDiff(equalTuples{numTuples},equalTuples{numTuples}(keepIdx))];
end

if n - numel(redundantObj) > 2
    independantObj = find(all(abs(PCorrelating-eye(n)) <= abs(paretoObj.config.independantCorrelation)));
end

paretoObj.status.independantObj = independantObj;
paretoObj.status.redundantObj = redundantObj;
paretoObj.status.conflictingObj = ParetoController.paretoSetDiff(paretoObj.status.conflictingObj,...
    [paretoObj.status.independantObj, paretoObj.status.redundantObj]);

end

%%
function tuple = getConnectedObjectives(currTuple, equal)
% Get tuples of connected objectives, e.g. (1,2), (2,3) -> (1,2,3)

tuple = currTuple;
checked = ismember(equal, currTuple);
toCheck = any(checked,2) & ~all(checked,2);
newAdded = unique(equal(toCheck & ~checked));
equal(all(checked,2),:) = [];
tuple = [tuple, reshape(newAdded,1,numel(newAdded))];
if any(toCheck,'all')
    tuple = getConnectedObjectives(tuple, equal);
end

end

%%
function [equalPairs, redundantObj, equalTuples] = eliminiateOverrepresentedObjectives(equalPairs, redundantObj)
% Count the occurrence of objectives in the tuples, the dominant ones are eliminated.

equalTemp = equalPairs(~any(ismember(equalPairs,redundantObj),2),:);
equalTuples = {}; % collection of connected objectives
% find tuples of connected objectives [2 3; 3 4] -> [2 3 4],
% but [1 2; 3 4] -> [1 2; 3 4]
while ~isempty(equalTemp)
    oneTuple = getConnectedObjectives(equalTemp(1,:), equalPairs);
    equalTemp(all(ismember(equalTemp,oneTuple),2),:) = [];
    equalTuples = [equalTuples; oneTuple];
end

occurrenceTuples = cell(size(equalTuples));

for tupleIdx = 1:length(equalTuples)
    for obj = 1:length(equalTuples{tupleIdx})
        occurrenceTuples{tupleIdx}(1,obj) = sum(equalPairs == equalTuples{tupleIdx}(1,obj),'all');
    end
    if ~all(occurrenceTuples{tupleIdx} == max(occurrenceTuples{tupleIdx}))
        redundantObj = [redundantObj, equalTuples{tupleIdx}(equalTuples{tupleIdx} == max(occurrenceTuples{tupleIdx}))];
    end
end

if ~all(cellfun(@(x) all(x == x(1)),occurrenceTuples))
    [equalPairs, redundantObj, equalTuples] = eliminiateOverrepresentedObjectives(equalPairs(...
        ~any(ismember(equalPairs,redundantObj),2),:), redundantObj);
end
end