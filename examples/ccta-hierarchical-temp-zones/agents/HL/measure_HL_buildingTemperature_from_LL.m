function measure_HL_buildingTemperature_from_LL(agent, simulation)
%HLMEASURESTATEFROMLL Set HL theta_b to weighted average from LL
%temperature zones
state_ll = simulation.agents.LowerLevel.history.x(:, end);

w_th = [0.1288    0.2658    0.1196    0.0579    0.1842    0.1842    0.0555    0.0013    0.0027];

% By default, callbackMeasureState()'s are only called after first time step, 
% so no need to check 
agent.history.x(2, end) = w_th * state_ll;

end