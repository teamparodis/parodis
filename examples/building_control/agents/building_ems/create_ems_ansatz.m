function [model, controller] = create_ems_ansatz(initialPeakGuess, weights, T_hor)
%CREATE_EMS_ANSATZ Summary of this function goes here
%   Detailed explanation goes here

%% Create Model and Controller instances
model = createModel( @ems_model_fun, T_hor, 1 );

% use controller = SymbolicController(1) if Pareto optimization is not wanted
controller = ParetoController( 1 );

% Use AWDS as front determination scheme 
controller.config.frontDeterminationScheme = 'AWDS'; 

%% Disturbance sources
% path to CSV files
csv_dir = string( fileparts(mfilename('fullpath')) ) + filesep + "../Data/" + filesep;

% real disturbance from CSV
controller.realDisturbanceSource = csv_dir + "disturbances.csv";

% predicted disturbance from a function that creates predictions on the fly
controller.predDisturbanceSource = @dist_prediction;

%% Define Constraints
n_bat = 7; % Number of batteries with bat_cap each
charge_max = 4.7; % maximum charging power for each battery in kW
bat_cap = 14;

% constraint on battery capacity to avoid damages
controller.addBoxConstraint("x", 1, 0.15*bat_cap*n_bat, 0.85*bat_cap*n_bat);

% hard constraints on building temperature
controller.addBoxConstraint("x", 2, 19, 23);

% input constraints
controller.addBoxConstraint("u", 1:4, [-1000; 0; 0; -440], [1000; 199; 600; 0]);

% rate constraints
controller.addDeltaConstraint("dx", 1, -n_bat*charge_max, n_bat*charge_max, 60);

%% Add Parameters necessary for cost functions

% FOR COMFORT COST FUNCTION
controller.addParam( "theta_ref", [1 1], 21, false );


% FOR MONETARY COST FUNCTION 
controller.addParam("peak_cost_factor", [1 1], 101, false);

% anonymous function to set initial peak guess externally
wrap_param_Pgrid_max = @(T, callingAgent, agents, numScenarios)( param_get_Pgrid_max(T, callingAgent, agents, numScenarios, initialPeakGuess) );
controller.addParam("Pgrid_max", [1 1], wrap_param_Pgrid_max, false);

% 0.131 €/kWh for buying, 0.07 €/kWh for selling energy
controller.addParam("price_pred", [1 length(T_hor)], repmat(0.131, 1, length(T_hor)), false);
controller.addParam("c_grid_sell", [1 1], 0.07, false);



%% Add Cost Functions 
% cost functions are added with weights, but they are arbitrary if
% ParetoController is used (and the cost function not explicitly neglected
% from Pareto optimization
controller.addCostFunction('monetary_costs', MonetaryCostFunction_Industry, weights(1) );
controller.addCostFunction('comfort_costs', ComfortCostFunction_TimeVarying, weights(2) );


end

