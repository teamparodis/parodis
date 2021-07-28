function [inputs, slacks, front, parameters] = determineFPBI(paretoObj, agent, optimizer, extremePoints, preselectedStartingPoint)
%Focus Point Boundary Intersection

if nargin == 4 || isempty(preselectedStartingPoint)
    
numEP = size(extremePoints,1);

inputs = [];
planePointsUsed = [];

normedEP = ParetoController.ParetoNormalization(extremePoints, paretoObj);

%calculate center point
[planePoints, ~] = getPlanePoints(paretoObj, normedEP(:,paretoObj.status.conflictingObj), paretoObj.status.focusVector, normedEP);

paretoObj.paretoMaxStep = size(planePoints,1)-1;

front = [];
inputs = {};
slacks = struct.empty;

for iParetoStep = 2:size(planePoints,1)
    paretoObj.paretoCurrentStep = iParetoStep - 1;
    agent.simulation.updateProgress();
    
    [optOut, feasibilityCode] = optimizer(planePoints(iParetoStep,:));
    
    if feasibilityCode ~= 0
        msg = "Yalmip error in Simulation step " + agent.sim.status.k + " in Pareto step " + paretoObj.paretoCurrentStep + ": " + (feasibilityCode);
        if paretoObj.config.printSolverStatus
            agent.log(msg); %possible: yalmiperror
        else
            warning(msg);
        end
        continue
    end
    
    [newPoint, newInput, newSlack] = calculateUnnormedObjectiveValues(...
        paretoObj, optOut, agent);
    
    if isequal(newPoint,[0 0 0])
        continue
    end
    
    front(end + 1,:) = newPoint;
    inputs{end + 1,1} = newInput;
    slacks = [slacks; newSlack];
    planePointsUsed = [planePointsUsed; planePoints(iParetoStep,:)];
end

paretoObj.status.nadir = max(front);
filteredFront = ParetoController.paretoFilter(paretoObj, front, 1:numEP);
front = front(filteredFront,:);

remainingIndices = filteredFront - numEP;
remainingIndices(remainingIndices <= 0) = [];
parameters = planePointsUsed(remainingIndices,:);
inputs = inputs(remainingIndices);
slacks = slacks(remainingIndices);

elseif nargin == 5
    optOut = optimizer(preselectedStartingPoint);
    
    [front, inputs, slacks] = calculateUnnormedObjectiveValues(paretoObj, optOut, agent);
else
    error("Input error in determineFPBI, not enough inputs.")
end

paretoObj.paretoCurrentStep = [];
paretoObj.paretoMaxStep = [];

end

%%
function [planePoints,dist] = getPlanePoints(paretoObj, surfacePoints, focusVector, normedEP)
% Calculate the evenly distributed points on the plane that are projected onto the Pareto front.

n = size(surfacePoints,2);

% find the indices of the farthest extreme points
largestDistanceIdc = extremePointsWithLargestDistance(normedEP);
referencePoint = largestDistanceIdc(2);

% the first direction in the sampled plane is the direction between the two farthest extreme points
planeDir = surfacePoints(ParetoController.paretoSetDiff(1:size(normedEP,1),largestDistanceIdc),:)-surfacePoints(referencePoint,:);
firstPlaneDir = [-1, 1]*surfacePoints(largestDistanceIdc,:);
L = norm(firstPlaneDir); % distance between the two farthest extreme points
orthonormalPlaneDir = firstPlaneDir/norm(firstPlaneDir);

% calculate the k-2 remaining plane directions
for i=1:size(planeDir,1)
    if size(orthonormalPlaneDir,1) == size(planeDir,1)-1
        orthonormalPlaneDir(end+1,:) = ParetoController.crossn([orthonormalPlaneDir; focusVector]);
    else
        orthonormalPlaneDir(end+1,:) = ParetoController.crossn([orthonormalPlaneDir; focusVector; planeDir(ParetoController.paretoSetDiff(i:size(planeDir,1)-1,referencePoint),:)]);
    end
end

orthonormalPlaneDir = orthonormalPlaneDir./vecnorm(orthonormalPlaneDir,2,2);

if size(orthonormalPlaneDir,1) >= 2
    L(1,2:size(orthonormalPlaneDir,1)) = L(1,1)*paretoObj.config.secondaryL;
end

dist = L(1,1)/paretoObj.config.m;
ndGridInput = {linspace(0,L(1,1),paretoObj.config.m)};
for i = 2:size(L,2)
    ndGridInput = [ndGridInput; linspace(-L(1,i)/2,L(1,i)/2,round(L(1,i)/dist))];
    if mod(round(L(1,i)/dist),2) == 0
        ndGridInput{i} = [ndGridInput{i},0];
    end
end
coordinates = cell(size(L,2),1);
[coordinates{:}] = ndgrid(ndGridInput{:});

cellOnes = num2cell(ones(1,n-1));
coordinatesMatrix = cell2mat(reshape(coordinates,cellOnes{:},length(coordinates)));
planePoints = [];

idxVec = num2cell(ones(1,n-1)); % vector of indices in coordinatesMatrix
finished = false;

% variable number of nested for-loops
while ~finished
    planePoints(end+1,:) = reshape(coordinatesMatrix(idxVec{:},:),1,n-1)*orthonormalPlaneDir;
    % increase values of indices until max is reached, then set to 1 and
    % increase next, until everything is checked.
    for pos = n-1:-1:1
        idxVec{pos} = idxVec{pos}+1;
        if idxVec{pos} > size(coordinatesMatrix,pos)
            idxVec{pos} = 1;
            if pos == 1
                finished = true;
            end
        else
            break;
        end
    end
end

planePoints = planePoints + surfacePoints(1,:);

end

%%
function idc = extremePointsWithLargestDistance(normedEP)
% find the indices of the two farthest extreme points
    combinations = nchoosek(1:size(normedEP,1),2);
    dist = nan(size(combinations,1),1);
    for i = 1:size(combinations,1)
        dist(i,1) = norm([1 -1]*normedEP(combinations(i,:),:));
    end
    idc = combinations(dist == max(dist),:);
end