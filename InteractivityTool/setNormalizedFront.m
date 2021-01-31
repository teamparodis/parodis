function setNormalizedFront(app)
% calculate normalized Pareto front
app.J_nadir = max(app.front);
app.J_utopia = min(app.front);
app.normalizedFront = (app.front-app.J_utopia)./(app.J_nadir-app.J_utopia);
app.frontToPlot = app.normalizedFront;
end