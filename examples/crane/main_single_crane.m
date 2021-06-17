%% This is the main script for the single crane example 

% Delete workspace, add paths
clear all;
close all;
yalmip('clear');

% time horizon vector in seconds
T_hor = repmat(0.5, 1, 10) ;

% 1) create model 

model = createModel( @model_crane_linear, T_hor, 1 )

