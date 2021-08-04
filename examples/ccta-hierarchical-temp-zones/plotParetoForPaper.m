frontsSorted = cellfun(@(x) sortrows(x,1),hlAgent.history.pareto.fronts,'uni',0);
fronts = cellfun(@(x) (x-min(x))./(max(x)-min(x)), frontsSorted, 'uni',0);
costs = hlAgent.history.pareto.chosenSolutions;
costsNormed = [];

for i = 1:length(fronts)
    costsNormed(end+1,:) = (costs(i,:)-min(frontsSorted{i}))./(max(frontsSorted{i})-min(frontsSorted{i}));
end

storingPath = ['./Results/' sim.simulationName '/plots/'];
%%
f1 = figure;
hold on
grid on
for i = 1:length(fronts)
    p = plot(fronts{i}(:,1),fronts{i}(:,2),'r');
    p.Color = [1 0 0 0.1];
end
s = scatter(costsNormed(:,1),costsNormed(:,2),'bx');

legend([p, s], {'Pareto Front', 'Selected Solution'}); 
s.MarkerEdgeAlpha = 0.2;
savefig(f1, [storingPath 'frontsAndCUP2'])
xlabel('Normalized Monetary Costs'); 
ylabel('Normalized Comfort Costs'); 
title('Generated Pareto Fronts');

%%
f2 = figure;
s = scatter(costsNormed(:,1),costsNormed(:,2),'bx');
s.MarkerEdgeAlpha = 0.3;
% savefig(f2, [storingPath 'CUP2'] )

f3 = figure;
hold on
grid on
for i = 1:length(fronts)
p = plot(fronts{i}(:,1),fronts{i}(:,2),'r');
p.Color = [1 0 0 0.1];
end
% savefig(f2, [storingPath 'fronts2'] )

%%
pointsPerFront = mean(cellfun(@(x) (numel(x)/2), fronts))
