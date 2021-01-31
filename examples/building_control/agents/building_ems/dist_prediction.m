function [dPred] = dist_prediction(T, ~, ~, ~)
%DIST_PREDICTION Predicts the disturbances for the current horizon

% T = 0 corresponds to midnight
% cycle time in 24h hours, since only relative time is relevant
T_cycle = mod(T, 24*60);

%round(T_cycle/60, 2)

%% first disturbance is P_ren, so energy from solar system
% assumption: sinusoid from 7 am to 8pm, peak 80kW, outside 0
T_period = (20-7)*60*2;
phi = 7*60;
Pren_peak = 80;

Pren = Pren_peak * max( sin(2*pi* (T_cycle - phi) / T_period ), 0 );

%% second disturbance is P_dem
% assumption: baseload at -100kW, during the day sin² with peaks at 9:30 am, and 3:00 pm at -200kW


% first peak: between 8:15 and 10:45 take sinus around 9:30, outside assume 0
T_period = 5*60;
phi1 = 8.25*60;
p1 = sin( 2*pi*(T_cycle - phi1)/T_period );
p1(T_cycle < 8.25*60) = 0;
p1(T_cycle > 10.75*60) = 0;

% second peak: between 13:45 and 16:15 take sinus around 15:00, outside assume 0
T_period = 5*60;
phi2 = 13.75*60;
p2 = sin( 2*pi*(T_cycle - phi2)/T_period );
p2(T_cycle < 13.75*60) = 0;
p2(T_cycle > 16.25*60) = 0;

% base load demand is around 100kW
Pdem_base = -100;

% peak demand is another 100kW on top
Pdem = Pdem_base - 100*(p1+p2);

%% third disturbance is outside temp theta_air
% assumption sinusoid around from 8 to 24°C, with 16°C at 10:30 am, period 24h
T_period = 24*60;
phi = 10.5*60;

theta_air = 16 + 8*sin( 2*pi*(T_cycle - phi) / T_period);

%% finally, join disturbances into matrix and wrap in cell
dPred = { [Pren; Pdem; theta_air] };

end