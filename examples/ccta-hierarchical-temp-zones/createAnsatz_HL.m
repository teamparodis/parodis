function [hlModel, hlController] = createAnsatz_HL(initialPeakGuess, T_s, weights)
%CREATE_EMS_ANSATZ Summary of this function goes here
%   Detailed explanation goes here

%% Create Model and Controller instances
hlModel = createModel( @model_HL, T_s );
hlController = ParetoController();
% hlController = SymbolicController(); uncomment to not use pareto optimization

%% Disturbance sources
% path to CSV files

csv_dir = string( fileparts(mfilename('fullpath')) ) + filesep + "/data/" + filesep;
hlController.realDisturbanceSource = csv_dir + "hl_disturbancesApril2020_15minSteps_ADJUSTED.csv";
% hlController.realDisturbanceSource = csv_dir + "hl_disturbancesApril2020_15minSteps_ADJUSTED_higherTemperature.csv";
hlController.predDisturbanceSource = hlController.realDisturbanceSource;

%% Define Constraints
n_bat = 7; % Number of batteries with bat_cap each
charge_max = 4.7; % maximum charging power for each battery in kW
bat_cap = 14;

% constraint on battery capacity to avoid damages
hlController.addBoxConstraint("x", 1, 0.15*bat_cap*n_bat, 0.85*bat_cap*n_bat);

% hard constraints on building temperature
hlController.addBoxConstraint("x", 2, 19, 23);

% input constraints
hlController.addBoxConstraint("u", 1:4, [-1000; 0; 0; -440], [1000; 199; 600; 0]);

% rate constraints
hlController.addDeltaConstraint("dx", 1, -n_bat*charge_max, n_bat*charge_max, 60);

%% Add Parameters necessary for cost functions

% FOR COMFORT COST FUNCTION
hlController.addParam( "theta_ref", [1 1], 21, false );


% FOR MONETARY COST FUNCTION 
hlController.addParam("peak_cost_factor", [1 1], 101, false);

% anonymous function to set initial peak guess externally
wrap_param_Pgrid_max = @(T, callingAgent, agents, numScenarios)( param_Pgrid_max(T, callingAgent, agents, numScenarios, initialPeakGuess) );

hlController.addParam("Pgrid_max", [1 1], wrap_param_Pgrid_max, false);

% 0.131 €/kWh for buying, 0.07 €/kWh for selling energy
hlController.addParam("price_pred", [1 length(T_s)], repmat(0.131, 1, length(T_s)), false);
hlController.addParam("c_grid_sell", [1 1], 0.07, false);


% FOR BATTERY DEGRADATION COST FUNCTION
hlController.addParam("inv_C_bat",       [1 length(T_s)+1],   repmat(1/(0.85*bat_cap*n_bat),1,length(T_s)+1), false);
hlController.addParam("inv_Pcharge_max", [1 length(T_s)],     repmat(1/(n_bat*charge_max),1,length(T_s)), false);
hlController.addParam("bat_lower_constraint", [1 1], 0.15*bat_cap*n_bat, false); 
hlController.addParam("Pcharge_max_in_kW", [1 1], n_bat*charge_max, false); 

%% Add Cost Functions 
if nargin < 6
    weights = [0.2 0.8];
end

hlController.addCostFunction('monetary_costs', Cost_HL_monetary_industry, weights(1) );
hlController.addCostFunction('comfort_costs', Cost_HL_comfort, weights(2) );


end

