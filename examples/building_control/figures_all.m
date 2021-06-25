% Plotting for building energy management system example

color1 = [76,136,255]./256;
color2 = [235,130,44]./256;
color3 = [67,9,77]./256;
color4 = [133,0,126]./256;

figTrajectories = TimeSeries("states", 4, 1);
figTrajectories.setFigureOptions({"Position", [85 270 630 735]});
% Subplot 1: SoC vs Electricity
figTrajectories.addLine(emsAgent, 'eval', 'E_max', 1, {'Limits'}, 1, {'Color', color1, 'LineStyle', '--'});
figTrajectories.addLine(emsAgent, 'x',     1,        1, {'E'}, 1, {'Color', color1, }, {}, 'left');
figTrajectories.addLine(emsAgent, 'eval', 'E_min', 1, {}, 1, {'Color', color1, 'LineStyle', '--'});
figTrajectories.setSubplotTitle(1, 'Battery Storage');
figTrajectories.setYLabel(1, 'Energy in kWh');

% Subplot 2: Theta vs Outside
figTrajectories.addLine(emsAgent, 'x', 2, 2, {'\theta_b'}, 1, {'Color', color1}, {}, 'left');
figTrajectories.addLine(emsAgent, 'd', 3, 2, {'\theta_{air}'}, 1, {'Color', color2, 'LineStyle', '--'}, {}, 'left');
figTrajectories.setYLabel(2, 'Temperature in °C', 'left');
tempLimits = [2, 25];
figTrajectories.setFixedYLimits(2, tempLimits, 'left');
figTrajectories.setSubplotTitle(2, 'Building vs. Air temperature');

% Subplot 3: Inputs
indSubInputs = 3; 
figTrajectories.addLine(emsAgent, 'u', 1, indSubInputs, {'P_{grid}'}, 1, {'Color', color1, 'LineStyle', '-'});
figTrajectories.addLine(emsAgent, 'u', 2, indSubInputs, {'P_{CHP}'}, 1,  {'Color', color2, 'LineStyle', '--'});
figTrajectories.addLine(emsAgent, 'u', 3, indSubInputs, {'Q_{rad}'}, 1,  {'Color', color3, 'LineStyle', '-.'});
figTrajectories.addLine(emsAgent, 'u', 4, indSubInputs, {'Q_{cool}'}, 1, {'Color', color4, 'LineStyle', ':'});

figTrajectories.setYLabel(indSubInputs, 'Power in kW', 'left');
figTrajectories.setSubplotTitle(indSubInputs, 'Inputs');

% Subplot 4: Disturbances
indSubDist = 4; 
figTrajectories.addLine(emsAgent, 'd', 1, indSubDist, {'P_{ren}'}, 1, {'Color', color1, 'LineStyle', '-'}, {'LineStyle', '-.'} );
figTrajectories.addLine(emsAgent, 'd', 2, indSubDist, {'P_{dem}'}, 1, {'Color', color2, 'LineStyle', '--'});
figTrajectories.setYLabel(indSubDist, 'Power in kW', 'left');
figTrajectories.setSubplotTitle(indSubDist, 'Disturbances');

figTrajectories.setXLabel(4, 'Time in min');
for i=1:4
    figTrajectories.setFixedXLimits(i, [0 T_sim]);
end
sim.addPlots(figTrajectories); 

%% Plot Battery charging power

fig3 = TimeSeries("other", 2, 1);
fig3.setFigureOptions({"Position", [730 435 560 420]});
% fig3.addLine(emsAgent, 'eval', 'Pcharge_max', 1, {'P_{charge,max}'}, 1, {'Color', 'r'});
% fig3.addLine(emsAgent, 'eval', 'Pcharge', 1, {'P_{charge}'});
% fig3.addLine(emsAgent, 'eval', 'Pcharge_min', 1, {'P_{charge,min}'}, 1, {'Color', 'r'});
% fig3.setXLabel([], 'Time in min');
% fig3.setYLabel(1, 'Power in kWh', 'left');
% fig3.setSubplotTitle(1, 'Charging Power');

fig3.addLine(emsAgent, 'x', 2, 1, {'Building Temperature'});
fig3.setYLabel(1, 'Temperature in °C', 'left');
fig3.setSubplotTitle(1, 'Building Temperature');
fig3.setFixedYLimits(1, [19 23]);

fig3.addLine(emsAgent, 'eval', 'monetary_costs', 2, {'Mon. Costs'});
fig3.setYLabel(2, 'Monetary costs in €', 'left');
fig3.setSubplotTitle(2, 'Monetary Costs');
fig3.setFixedYLimits(2, [0 450]);

fig3.setFixedXLimits(1, [0 T_sim]);
fig3.setFixedXLimits(2, [0 T_sim]);

sim.addPlots(fig3);
