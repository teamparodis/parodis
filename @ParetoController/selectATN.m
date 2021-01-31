function [idx,utility] = selectATN(varargin)
%AEP: Metric function that minimizes the angle to all EP.
%   Calculates the angle as the opening angle of a hyper cone of every
%   non-extreme point on the Pareto front to the extreme points.
% INPUT: varargin is either a Pareto front (numeric) or it can be a ParetoController object and a
%   timestep for the interactivity tool  
if isa(varargin{1},'numeric')
    front = varargin{1};
    normedFront = (front-min(front))./(max(front)-min(front));
elseif isa(varargin{1},'Agent') && length(varargin(:)) >= 2 
    front = varargin{1}.history.pareto.fronts{varargin{2}};
    normedFront = (front-min(front))./(max(front)-min(front));
else
    error("Input has to be either a Pareto front or an Agent object with a timestep.")
end

dim = size(normedFront,2);

[~,EP_pos] = max(normedFront);
BP = ParetoController.getBorderPoints(normedFront);

utilityATN = NaN(size(normedFront,1),1);

for j=ParetoController.paretoSetDiff(1:size(normedFront,1),[EP_pos,BP])  % skip extreme points
    AP = ParetoController.getAdjacentPoints(normedFront, normedFront(j,:), BP);
    if numel(AP) < dim
        continue;
    end
    utilityATN(j,1) = coneAngle(normedFront(j,:), normedFront(AP,:));
end

utility = utilityATN;
[~, idx] = min(utilityATN);

end

%%

function [angle] = coneAngle(pfPoint, conePoints)
%coneAngle: calculates the opening angle of a hyper-cone in n-dimensions
%   pfPoint: One point on the Pareto front
%   conePoints: other points on the Pareto front, in this case the EP
%   angle: opening angle of the n-dimensional hyper-cone -> min. for
%   solution

dim = size(pfPoint,2);
numPoints = size(conePoints,1);

if (dim ~= numPoints)
    error("Wrong number of cone points!")
end

coneBasePlane = zeros(numPoints-1,size(conePoints,2));

% calculate vectors spanning the cone's mantle
mantleVectors = conePoints - pfPoint;
mantleVectorsNorm = mantleVectors./vecnorm(mantleVectors,2,2);

% get the vectors spanning the cone's base
for i=2:numPoints
    coneBasePlane(i-1,:) = mantleVectorsNorm(i-1,:) - mantleVectorsNorm(i,:);
end

% get the cone's axis direction as the nullspace of the base
coneAxis = null(coneBasePlane);

% the opening angle is two times the angle between one mantle vector and
% the axis

angle = 2*acos(mantleVectorsNorm(1,:)*coneAxis);

end

