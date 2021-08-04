%% This is the main script for the example from the CCTA 2021 Paper

clear all;
close all;
yalmip('clear');

cd(fileparts(mfilename('fullpath')));
addpath(genpath(pwd));

T_sim = 7*24*60; % 7 days simulation

% 24 hour horizon with first 8 h in 15 min steps, second 8 h in 30 min
% steps, last 8 h in 60 min steps

T_s = [repmat(15, 1, 4*8), repmat(30, 1, 2*8), repmat(60, 1, 1*8)];

% initially assumed peak power demand
initialPeak = 200; 

%% Higher Level Agent

% create_hl_ansatz(implicitPrediction, T_sim, scenario, day_begin, initialPeakGuess, weights, T_s)
[hlModel, hlController] = createAnsatz_HL(initialPeak, T_s);
hlAgent = Agent( 'HigherLevel', hlModel, hlController, T_s, [49 21]');
hlAgent.callbackMeasureState = @measure_HL_buildingTemperature_from_LL;
hlAgent.config.solver =  'gurobi'; %'sdpt3'; %'sedumi', 'sdpt3'; %'quadprog';
%     hlAgent.config.disturbanceMeasured = 1;

% Add some evaluation functions for plotting purposes
hlAgent.addEvalFunction( 'Pcharge', @eval_Pcharge, false);
hlAgent.addEvalFunction( 'Pcharge_min', eval_const(-32.9));
hlAgent.addEvalFunction( 'Pcharge_max', eval_const( 32.9));
hlAgent.addEvalFunction( 'electricity_costs', @eval_electricity_costs);

hlAgent.addEvalFunction( 'E_min', eval_const(14.7));
hlAgent.addEvalFunction( 'E_max', eval_const(83.3));

hlAgent.addEvalFunction( 'PeffectiveLoad', @eval_PeffectiveLoad, false);


%% Lower Level Agent

[llModel, llController] = createAnsatz_LL(false, T_sim, [], T_s);
llAgent = Agent( 'LowerLevel', llModel, llController, T_s, [repmat(21, 9, 1)]);

%% Simulation instance
sim = Simulation( 'ccta-hierarchical-ems', T_sim );
sim.config.livePlot = false;

sim.addAgents(hlAgent, llAgent); 

% common_figures_HL
% common_figures_LL
figures_all_ab

sim.runSimulation();