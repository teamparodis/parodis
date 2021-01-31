function openInteractivityTool(sim, agent)
%OPENINTERACTIVITYTOOL Summary of this function goes here
%   Detailed explanation goes here
if ~isequal(agent.controller.type,'pareto') 
    error("The agent's controller is not a ParetoController.");
else
switch length(agent.controller.costFunctions)
    case 1
        error("There is no pareto front with only one objective!")
    case 2
        ParetoInteractivity2D(sim);
    case 3
        ParetoInteractivity3D(sim);
    otherwise
        error("Let's be honest: We didn't find a way (yet) to show you n-dimensional fronts.");
end
end

