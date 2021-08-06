function [ Pgrid_max ] = param_get_Pgrid_max(T, callingAgent, agents, numScenarios, initialPeakGuess)
%PARAM_GET_U_MAX Retrieves the peak energy drawn from the grid so far
if nargin < 5
    initialPeakGuess = 408;
end

if size(callingAgent.history.u, 2) > 0
    Pgrid_max = max(callingAgent.history.u(1, :));
else
    Pgrid_max = 0;
end

% wrap into cell array
Pgrid_max = max(initialPeakGuess, Pgrid_max);
Pgrid_max = repmat( {Pgrid_max}, numScenarios, 1 );