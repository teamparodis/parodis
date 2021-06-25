%% Add pareto plot for EMS

figPareto = ParetoPlot("pareto1", 1, 1);

% addParetoFront(agent, objectives, subplotIndex, cumulative,  ...
% [optional]:    normalized, showLegend, optionsFront, optionsSolution,

% figPareto.addParetoFront(emsAgent, [1, 2], 1, 0, 1, 1, {'LineWidth', 2}, {'MarkerFaceColor', [0.91797,0.50781,0.17188]}) ;
% figPareto.setSubplotTitle(1, 'Current Pareto Front');

figPareto.addParetoFront(emsAgent, [1, 2], 1, 1, 1, 1, {'LineWidth', 2}) ;
figPareto.setSubplotTitle(1, 'All Pareto Fronts (cumulative)');

sim.addPlots(figPareto);
