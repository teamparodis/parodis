function idc = paretoFilter(paretoObj, inputPts, fixedPointIdc)
%Pareto filter that tries to filter out weakly Pareto optimal solutions.
% INPUT:
%   paretoObj: ParetoController object
%   inputPts: all possible Pareto optimal points
%   fixesPointIdc: indices of points that are definetly Pareto optimal, e.g. the extreme points

idxPO = [];
potentialPts = (inputPts-min(inputPts))./(max(inputPts)-min(inputPts));
for i=1:size(potentialPts,1)
    diff = potentialPts-potentialPts(i,:);
    diffTol = diff;
    diffTol(i,:) = nan(1, size(diff,2));
    diffTolBound = paretoObj.config.diffTolBound;
    diffTol(diffTol > -diffTolBound & diffTol < diffTolBound) = 0;
    s = sign(diffTol);
    
    if any(all(s<=0,2) & any(s==-1,2))
        continue;
    elseif any(all(s==0,2))
        [~,minInd] = min(vecnorm(potentialPts([i; find(all(s==0,2))],:),2,2));
        if minInd ~= 1
            continue;
        end
    end
    
    idxPO = [idxPO, i];
end

idc = [fixedPointIdc, ParetoController.paretoSetDiff(idxPO,fixedPointIdc)];
end