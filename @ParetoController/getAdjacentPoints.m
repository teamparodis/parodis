function [adjacentPts] = getAdjacentPoints(front, examinedPoint, ignorePoints)
%Calculates the neighbor points to examinedPoint on the front. Each possible spacial direction is
%calculated and the closest point in this direction is selected as one neighbor point.
% INPUT:
%   front: Pareto front
%   examinedPoint: point to find neighbors to
%   ignorePoints: indices of points that are not selected as neighbors

switch nargin
    case 2
        ignorePoints = [];
    case 1
        error("Not enough input arguments.")
end

adjacentPts = [];

frontVectors = front-examinedPoint;
frontVectors(all(frontVectors == 0,2), :) = NaN;

if (~isempty(ignorePoints))
    frontVectors(ignorePoints,:) = NaN;
end

% general directions in the space are calculated, e.g. [1 1; 1 -1; -1 1]
directionsOriginal = sign(frontVectors);
directions = directionsOriginal;

% make similar directions equal, e.g. [1 -1] and [-1 1] are equal
if size(frontVectors,2) > 2
    directions(sum(directions,2)<0,:) = -directions(sum(directions,2)<0,:);
    directions(sum(directions,2)==0,:) = directions(sum(directions,2)==0,1).*directions(sum(directions,2)==0,:);
end
dirTypes = unique(directions,'rows');
dirTypes = dirTypes(~isnan(dirTypes(:,1)),:);

if size(dirTypes,1) < size(dirTypes,2) % if not enough directions remain take all possible
    dirTypes = unique(directionsOriginal,'rows');
    dirTypes = dirTypes(~isnan(dirTypes(:,1)),:);
    directions = directionsOriginal;
end

for k=1:(min(size(dirTypes)))
    dirIdx = find(ismember(directions,dirTypes(k,:),'rows'));
    tempFront = frontVectors;
    tempFront(ParetoController.paretoSetDiff(1:size(tempFront,1),dirIdx),:) = NaN;
    [~, adjacentPts(k)] = min(vecnorm(tempFront,2,2));
end
end
