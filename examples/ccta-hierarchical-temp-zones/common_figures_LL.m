% % Figure 1: States
fig5 = TimeSeries("all", 1, 1);
fig5.setXLabel([], 'Time in min');
fig5.setFixedXLimits(1, [0 T_sim]);

%   Subplot 1: all states
fig5.addLine(llAgent, 'x', [], 1, {'\theta_1', '\theta_2', '\theta_3', '\theta_4', ...
    '\theta_5', '\theta_6', '\theta_7', '\theta_8', '\theta_9', },...
    1, {}, {}, 'left');
fig5.setYLabel(1, 'Temperature in °C', 'left');
fig5.setFixedYLimits(1, [19 23], 'left');
fig5.setSubplotTitle(1, 'Zone Temperatures');


% % Figure 2: Inputs
fig6 = TimeSeries("inputs", 2, 1); 
fig6.setXLabel([], 'Time in min');
fig6.setFixedXLimits(1, [0 T_sim]);

%   Subplot 1: Heating
fig6.addLine(llAgent, 'u', [1:9], 1, ...
   {'Q_heat_1', 'Q_heat_2', 'Q_heat_3', 'Q_heat_4', ...
    'Q_heat_5', 'Q_heat_6', 'Q_heat_7', 'Q_heat_8', 'Q_heat_9', },...
    1, {}, {}, 'left');
fig6.setYLabel(1, 'Heating Power in kW', 'left');
fig6.setFixedYLimits(1, [0 600], 'left');
fig6.setSubplotTitle(1, 'Heating ');

%   Subplot 2: Cooling
fig6.addLine(llAgent, 'u', [10:18], 2, {'Q_cool_1', 'Q_cool_2', 'Q_cool_3', 'Q_cool_4', ...
    'Q_cool_5', 'Q_cool_6', 'Q_cool_7', 'Q_cool_8', 'Q_cool_9', },...
    1, {}, {}, 'left');
fig6.setYLabel(2, 'Cooling Power in kW', 'left');
fig6.setFixedYLimits(2, [-440 0], 'left');
fig6.setSubplotTitle(2, 'Cooling');


% % % Figure 3: Q_i_multiplied
% fig7 = TimeSeries("eval", 1, 1);
% fig7.setXLabel([], 'Time in min');
% fig7.setFixedXLimits(1, [0 T_sim]);
% 
% %   Subplot 1: all states
% fig7.addLine(llAgent, 'eval', 'QiMultiplied', 1, {'Q_{heat1}*Q_{cool1}'});
% fig7.setYLabel(1, 'in kW^2');
% % fig7.setFixedYLimits(1, [19 23], 'left');
% fig7.setSubplotTitle(1, 'Evaluation what goes wrong...');


sim.addPlots(fig5, fig6);

