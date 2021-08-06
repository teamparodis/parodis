fig1 = TimeSeries("states", 2, 1);
% SoC vs Electricity
fig1.addLine(hlAgent, 'eval', 'SoC_max', 1, {}, 1, {'Color', 'r'});
fig1.addLine(hlAgent, 'x', 1, 1, {'SoC'}, 1, {}, {}, 'left');
fig1.addLine(hlAgent, 'eval', 'SoC_min', 1, {}, 1, {'Color', 'r'});
fig1.setSubplotTitle(1, 'State of Charge vs Electricity Price');

fig1.addLine(hlAgent, 'eval', 'electricity_costs', 1, {'Price'}, 1, {}, {}, 'right');
fig1.setYLabel(1, 'SoC in kWh', 'left');
fig1.setYLabel(1, 'Electricity price in €/kWh', 'right');
fig1.setFixedYLimits(1, [0 100], 'left');

% Theta vs Outside
fig1.addLine(hlAgent, 'x', 2, 2, {'\theta_b'}, 1, {}, {}, 'left');
fig1.addLine(hlAgent, 'd', 3, 2, {'\theta_{air}'}, 1, {}, {}, 'right');
fig1.setYLabel(2, 'Temperature in °C', 'left');
fig1.setYLabel(2, 'Temperature in °C', 'right');
fig1.setXLabel([], 'Time in min');
fig1.setFixedXLimits(1, [0 T_sim]);
fig1.setFixedXLimits(2, [0 T_sim]);
fig1.setFixedYLimits(2, [19 23], 'left');
fig1.setSubplotTitle(2, 'Building vs. Air temperature');

fig2 = TimeSeries("u and d", 2, 1);
fig2.addLine(hlAgent, 'u', 1:4, 1, {'P_{grid}', 'P_{CHP}', 'Q_{rad}', 'Q_{cool}'});
fig2.addLine(hlAgent, 'd', [1], 2, {'P_{ren}'}, 1, {'LineStyle', '-.'}, {'LineStyle', '-.'} );
fig2.addLine(hlAgent, 'd', [2], 2, {'P_{dem}'});

fig2.addLine(hlAgent, 'd', 3, 2, {'\theta_{air}'}, 1, {}, {}, 'right');
fig2.setXLabel([], 'Time in min');
fig2.setYLabel(1, 'Power in kW', 'left');
fig2.setYLabel(2, '(Electrical) Power in kW', 'left');
fig2.setYLabel(2, 'Temperature in °C', 'right');
fig2.setFixedXLimits(1, [0 T_sim]);
fig2.setFixedXLimits(2, [0 T_sim]);
fig2.setSubplotTitle(1, 'Inputs');
fig2.setSubplotTitle(2, 'Disturbances');

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


fig4 = TimeSeries("Effective Load", 1, 1); 
fig4.addLine(hlAgent, 'eval', 'PeffectiveLoad', 1, {'P_{ren} + P_{dem}'}, 1); 
fig4.setYLabel(1, 'Power in kW'); 
fig4.setXLabel([], 'Time in min'); 
fig4.setFixedXLimits(1, [0 T_sim]);

% sim.addPlots(fig1, fig2, fig3, fig4);
sim.addPlots(fig1, fig2);