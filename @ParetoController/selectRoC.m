function [idx, utility] = selectRoC(varargin)
%ROC: Calculates the radius of curvature for finding existing knee points
%on the Pareto Front
%   radius is the radius of curvature on the PF to it's neighbors.
%   Direction has the information, if the center lies left or right of the
%   PF point.
%   Direction:  +1: The center points lies in positive objective direction
%               -1: The center points lies in negative objective direction
%                0: Not evalutated points
%   dir_mode: true or false
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
[~, EP_pos] = max(normedFront);
radius = [];
BP = ParetoController.getBorderPoints(normedFront);
radius = NaN(size(normedFront,1),1);
center = [];

for iPoints = ParetoController.paretoSetDiff(1:length(normedFront),[EP_pos,BP])
    AP = ParetoController.getAdjacentPoints(normedFront, normedFront(iPoints,:), BP);
    
    if length(AP) == dim
        center(iPoints,:) = (2*(normedFront(AP,:)-normedFront(iPoints,:))\((vecnorm(normedFront(AP,:),2,2).^2-vecnorm(normedFront(iPoints,:),2,2)^2)))';
    elseif length(AP) > dim
        M = 2*(normedFront(AP,:)-normedFront(iPoints,:));
        M_weight = M'*diag(1./vecnorm((normedFront(AP,:)-normedFront(iPoints,:)),Inf,2));
        center(iPoints,:) = ((M_weight*M)\(M_weight)*((vecnorm(normedFront(AP,:),2,2).^2-vecnorm(normedFront(iPoints,:),2,2)^2)))';
    else
        continue;
    end
    
    radius(iPoints,:) = norm(normedFront(iPoints,:)-center(iPoints,:));
end

concave = all(center <= normedFront, 2);
radius(concave) = NaN;

[~,idx] = min(radius);
utility = radius;

end

