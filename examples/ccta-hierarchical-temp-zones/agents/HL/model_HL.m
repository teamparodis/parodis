function [ode, n_x, n_u, n_d, linearRepresentation] = model_HL( T_s )
%MODEL_HL_FUN Creates the ODEs for the higher level model
% States: SoC (stationary battery) and building temperature theta
% Inputs: P_grid, P_chp, Q_rad, Q_cool
% Disturbances: P_ren, P_dem, theta_a (outside air temperature)

% Set constants of differntial equations

H_a = 34193.8469*1e-3; % heat transfer coefficient. *1e-3 for kW/K instead of W/K
C_th = 1792064*1e-3;%*60; % thermal capacity. *1e-3 for kW instead of W, NOT*60 because it was given in Wh, not Wmin (?) % 1792064 in Wh/K

c_chp = 0.677; % 'current constant', so that P_chp = c_chp * Q_h_chp
eps_c = 2.5; % 'energy efficiency ratio' (EER) of electric chillers

DT = T_s/60;

% Continuous model
A_cont = [0, 0; 
         0 -H_a/C_th];
B_cont = [1 1 0 +1/eps_c; 
         0 1/(c_chp*C_th) 1/C_th 1/C_th];
S_cont = [1 1 0;
        0 0 H_a/C_th];
% Discretize model using control toolbox 

%% Discretize System(s)

Adis = expm(A_cont*DT); 

h1 = @(tau) expm(A_cont*(DT-tau))*B_cont;
Bdis = integral(h1, 0, DT, 'ArrayValued', true);

h2 = @(tau) expm(A_cont*(DT-tau))*S_cont;
Sdis = integral(h2, 0, DT, 'ArrayValued', true);

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
