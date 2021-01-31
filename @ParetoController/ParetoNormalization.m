function normalizedPoints = ParetoNormalization(points, paretoObj)
% Normalize the given Pareto optimal points using the utopia and nadir points or the utopia point
% and a fix max value.
% INPUTS:
%   points: Pareto-optimal points to normalize
%   paretoObj: ParetoController instance to get type of normalization and Utopia and Nadir point

if nargin == 1 || isequal(paretoObj.config.normalization, 'simple')
    % normalization using the max and min values of each objective
    normalizedPoints = (points - min(points,[],1))./(max(points,[],1) - min(points,[],1));
elseif isequal(paretoObj.config.normalization, 'dynamic') || length(paretoObj.config.fixedNormValues) ~= length(points)
    % normalization using Utopia and Nadir point
    normalizedPoints = (points - paretoObj.status.utopia)./(paretoObj.status.nadir - paretoObj.status.utopia);
elseif isequal(paretoObj.config.normalization, 'fixed')
    % normalization using Utopia point and a fixed, preselected guess for the Nadir point
    normalizedPoints = (points - paretoObj.status.utopia)./(paretoObj.config.fixedNormValues - paretoObj.status.utopia);
else
    normalizedPoints = (points - paretoObj.status.utopia)./(paretoObj.status.nadir - paretoObj.status.utopia);
    warning("Normalization method not recognized, will apply dynamic normalization. Try 'dynamic' or 'fixed'.")
end