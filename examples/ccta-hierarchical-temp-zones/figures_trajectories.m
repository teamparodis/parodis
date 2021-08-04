%Plotting of HL and LL trajectories as in CCTA 2021 Paper

color1 = [76,136,255]./256;
color2 = [235,130,44]./256;
color3 = [67,9,77]./256;
color4 = [133,0,126]./256;

%% LOWERLEVEL: heatmap for temperature zones

figHeatMapLL = TimeSeries("Heatmap_Thetai", 3, 1);

figHeatMapLL.addHeatmap(llAgent, 'x', [], 1, [], {}, {}); 
figHeatMapLL.setYLabel(1, 'Zone Index');
figHeatMapLL.setFixedXLimits(1, [0 T_sim]);
figHeatMapLL.setColorBarLabel(1, 'Temperature in °C');
figHeatMapLL.setSubplotTitle(1, 'Zone Temperatures \theta_{b,i}');

figHeatMapLL.addHeatmap(llAgent, 'u', [1:9], 2, [], {}, {}); 
figHeatMapLL.setXLabel([2], 'Time in min');
figHeatMapLL.setYLabel(2, 'Zone Index');
figHeatMapLL.setFixedXLimits(2, [0 T_sim]);
figHeatMapLL.setColorBarLabel(2, 'Thermal Power in kW');

figHeatMapLL.setSubplotTitle(2, 'Heatings Q_{heat,i}');

figHeatMapLL.addHeatmap(llAgent, 'u', [10:18], 3, [], {}, {}); 
figHeatMapLL.setXLabel(3, 'Time in min');
figHeatMapLL.setYLabel(3, 'Zone Index');
figHeatMapLL.setFixedXLimits(3, [0 T_sim]);
figHeatMapLL.setColorBarLabel(3, 'Thermal Power in kW');

figHeatMapLL.setSubplotTitle(3, 'Cooling Q_{cool,i}');


sim.addPlots(figHeatMapLL); 

%% higher level


figTrajectoriesHL = TimeSeries("states", 4, 1);

% Subplot 1: SoC vs Electricity
figTrajectoriesHL.addLine(hlAgent, 'eval', 'E_max', 1, {'Limits'}, 1, {'Color', color1, 'LineStyle', '--'});
figTrajectoriesHL.addLine(hlAgent, 'x',     1,        1, {'E'}, 1, {'Color', color1, }, {}, 'left');
figTrajectoriesHL.addLine(hlAgent, 'eval', 'E_min', 1, {}, 1, {'Color', color1, 'LineStyle', '--'});
figTrajectoriesHL.setSubplotTitle(1, 'Battery Storage');
figTrajectoriesHL.setFixedXLimits(1, [0 T_sim]);
figTrajectoriesHL.setYLabel(1, 'Energy in kWh');

% Subplot 2: Theta_b vs. Theta_s vs Outside
figTrajectoriesHL.addLine(hlAgent, 'x', 2, 2, {'\theta_{b}'}, 1, {'Color', color1}, {}, 'left');
figTrajectoriesHL.addLine(hlAgent, 'd', 3, 2, {'\theta_{a}'}, 1, {'Color', color2, 'LineStyle', '--'}, {}, 'left');
figTrajectoriesHL.setYLabel(2, 'Temperature in °C', 'left');
figTrajectoriesHL.setFixedXLimits(2, [0 T_sim]);
tempLimits = [2, 25];
figTrajectoriesHL.setFixedYLimits(2, tempLimits, 'left');
figTrajectoriesHL.setSubplotTitle(2, 'Building vs. Air temperature');

% Subplot 3: Inputs
indSubInputs = 3; 
figTrajectoriesHL.addLine(hlAgent, 'u', 1, indSubInputs, {'P_{grid}'}, 1, {'Color', color1, 'LineStyle', '-'});
figTrajectoriesHL.addLine(hlAgent, 'u', 2, indSubInputs, {'P_{chp}'}, 1,  {'Color', color2, 'LineStyle', '--'});
figTrajectoriesHL.addLine(hlAgent, 'u', 3, indSubInputs, {'Q_{rad}'}, 1,  {'Color', color3, 'LineStyle', '-.'});
figTrajectoriesHL.addLine(hlAgent, 'u', 4, indSubInputs, {'Q_{cool,b}'}, 1, {'Color', color4, 'LineStyle', ':'});

figTrajectoriesHL.setYLabel(indSubInputs, 'Power in kW', 'left');
figTrajectoriesHL.setSubplotTitle(indSubInputs, 'Inputs');

% Subplot 4: Disturbances
indSubDist = 4; 
figTrajectoriesHL.addLine(hlAgent, 'd', 1, indSubDist, {'P_{ren}'}, 1, {'Color', color1, 'LineStyle', '-'}, {'LineStyle', '-.'} );
figTrajectoriesHL.addLine(hlAgent, 'd', 2, indSubDist, {'P_{dem}'}, 1, {'Color', color2, 'LineStyle', '--'});
figTrajectoriesHL.setYLabel(indSubDist, 'Power in KW', 'left');
figTrajectoriesHL.setSubplotTitle(indSubDist, 'Disturbances');

figTrajectoriesHL.setXLabel(4, 'Time in min');

sim.addPlots(figTrajectoriesHL); 

%% Plot Battery charging power

fig3 = TimeSeries("other", 2, 1);
fig3.addLine(hlAgent, 'eval', 'Pcharge_max', 1, {'P_{charge,max}'}, 1, {'Color', 'r'});
fig3.addLine(hlAgent, 'eval', 'Pcharge', 1, {'P_{charge}'});
fig3.addLine(hlAgent, 'eval', 'Pcharge_min', 1, {'P_{charge,min}'}, 1, {'Color', 'r'});
fig3.setXLabel([], 'Time in min');
fig3.setYLabel(1, 'Power in kWh', 'left');
fig3.setSubplotTitle(1, 'Charging Power');

fig3.addLine(hlAgent, 'cost', 'monetary_costs', 2, {'Mon. Costs'});
fig3.setYLabel(2, 'Monetary costs in €', 'left');
fig3.setSubplotTitle(2, 'Monetary Costs');

fig3.setFixedXLimits(1, [0 T_sim]);
fig3.setFixedXLimits(2, [0 T_sim]);

sim.addPlots(fig3);
