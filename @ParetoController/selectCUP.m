function [idx,utility] = selectCUP(varargin)
%CUP: Closest to utopia point
%   Select the pareto-optimal point with the minimal distance to the utopia
%   point
% INPUT: varargin is either a Pareto front (numeric) or it can be a Agent object and a
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

utility = vecnorm(normedFront,2,2);
[~,idx] = min(utility);
end

