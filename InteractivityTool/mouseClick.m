function mouseClick(app,~,src2)
% select a point on the pareto front with a mouse click
[~,selectedIdx] = min(vecnorm(app.frontToPlot-src2.IntersectionPoint(:,1:2),2,2));
if selectedIdx == app.chosenIdx
    app.chosenIdx = [];
else
    app.chosenIdx = selectedIdx;
end
app.plotFront;
app.fillValueTable;
end