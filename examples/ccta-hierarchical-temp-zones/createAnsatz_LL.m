function [llModel, llController] = createAnsatz_LL_TEMP_b_1(T_s)
%CREATE_EMS_ANSATZ Summary of this function goes here
%   Detailed explanation goes here

%% Create Model and Controller instances
llModel = createModel( @model_LL, T_s );
llController = SymbolicController();

%% Disturbance sources
% path to CSV files
csv_dir = string( fileparts(mfilename('fullpath')) ) + filesep + "/data/" + filesep;

llController.realDisturbanceSource = csv_dir + "ll_disturbancesApril2020_15minSteps_ADJUSTED.csv";
% llController.realDisturbanceSource = csv_dir + "ll_disturbancesApril2020_15minSteps_ADJUSTED_higherTemperature.csv";
llController.predDisturbanceSource = llController.realDisturbanceSource;


%% Define Constraints

% define parameters for constraints from HL
QheatFromHL = llController.addParam( "QheatFromHL", [1 length(T_s)], @param_QheatFromHL, false );
QcoolFromHL = llController.addParam( "QcoolFromHL", [1 length(T_s)], @param_QcoolFromHL, false );

% hard constraints on building temperature
llController.addBoxConstraint("x", 1:9, repmat(19, 9, 1), repmat(23, 9, 1) );

% input constraints
llController.addBoxConstraint("u", 1:9,   zeros( 9, 1),    repmat(893.95, 9, 1) );  % on Q_heat_i
llController.addBoxConstraint("u", 10:18, repmat(-440, 9, 1), zeros(9, 1) );    % on Q_cool_i


% llController.addConstraint( ... 
%     (sum(llModel.u(1:9)) <= 600 + 199/0.677 ):'sum Q_heat_i <= 893.95 kW' ...
% ); 

llController.addConstraint( ... 
    ( sum(llModel.u(1:9,:), 1) ==  QheatFromHL{1} ):'sum Q_heat_i == Q_rad+P_chp/c_chp' ...
); 

% llController.addConstraint( ... 
%     (-440 <= sum(llModel.u(10:18)) ):'sum Q_cool_i >= -440 kW' ...
% ); 

llController.addConstraint( ... 
    ( sum(llModel.u(10:18,:), 1) ==  QcoolFromHL{1} ):'sum Q_cool_i == Q_rad+P_chp/c_chp' ...
); 


%% Add Parameters necessary for cost functions

% FOR COMFORT COST FUNCTION
llController.addParam( "theta_ref", [1 1], 21, false );


%% Add Cost Functions 


llController.addCostFunction('comfort_costs', Cost_LL_comfort);


end

