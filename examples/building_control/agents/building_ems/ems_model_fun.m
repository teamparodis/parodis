function [ode, n_x, n_u, n_d, linearRepresentation] = ems_model_fun( T_s )
%EMS_MODEL_FUN Creates the ODEs for the building model
% States: SoC (stationary battery) and building temperature theta
% Inputs: P_grid, P_chp, Q_rad, Q_cool
% Disturbances: P_ren, P_dem, theta_a (outside air temperature)

% Set constants of differntial equations

H_a = 34193.8469*1e-3; % heat transfer coefficient. *1e-3 for kW/K instead of W/K
C_th = 1792064*1e-3;%thermal capacity. *1e-3 for kW instead of W

c_chp = 0.677; % 'current constant', so that P_chp = c_chp * Q_h_chp
eps_c = 2.5; % 'energy efficiency ratio' (EER) of electric chillers

DT = T_s/60;

Adis = [ 1, 0; 
         0, exp(-(H_a*DT)/C_th)];
Bdis = [ DT, DT,  0, DT/eps_c; 
         0, -(exp(-(H_a*DT)/C_th)-1)/(H_a*c_chp), -(exp(-(H_a*DT)/C_th)-1)/H_a, -(exp(-(H_a*DT)/C_th)-1)/H_a];
Sdis = [ DT, DT, 0;
         0, 0, 1-exp(-(H_a*DT)/C_th)];

ode = @(x, u, d)( Adis*x +  Bdis*u +  Sdis*d);

% Set output parameters
n_x = length(Adis);
n_u = size(Bdis, 2);
n_d = size(Sdis, 2);

% Additional: linear representation for speed up
linearRepresentation = struct;
linearRepresentation.A = Adis;
linearRepresentation.B = Bdis;
linearRepresentation.S = Sdis;


