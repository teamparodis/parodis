%% This is the main script for the example of a building control

% Delete workspace, add paths
clear all;
close all;
yalmip('clear');

% add agents directory to the path
addpath( genpath( [pwd '/agents/'] ) );

% Simulation time in minutes: Something between 0 and 2 days
T_sim = 1*24*60;

% 24 hour horizon with first 8 h in 15 min steps, second 8 h in 30 min
% steps, last 8 h in 60 min steps
T_hor = [repmat(15, 1, 4*8), repmat(30, 1, 2*8), repmat(60, 1, 1*8)];

% P_grid above initial peak is subject to high peak costs.
initialPeak = 100;

% weighting for l_mon and l_comf if Pareto should *not* be used
weights = [0.5, 0.5];

%% Creating the agent

% definition of model and controller in separate function
[model, controller] = create_ems_ansatz(initialPeak, weights, T_hor);

emsAgent = Agent( 'building_ems', model, controller, T_hor, [49 21]');
% we can measure the current disturbance on the system, i.e. outside temperature, power demand, solar irradation
% with this flag dPred(k) = dReal(k), for k = the current time step
emsAgent.config.disturbanceMeasured = true;

emsAgent.config.solver =  'gurobi'; % other options: 'sdpt3'; 'sedumi'; 'quadprog';

% Add some evaluation functions for plotting purposes
% (not necessary for simulation)
emsAgent.addEvalFunction( 'Pcharge', @eval_Pcharge, false);
emsAgent.addEvalFunction( 'Pcharge_min', eval_const(-32.9));
emsAgent.addEvalFunction( 'Pcharge_max', eval_const( 32.9));
emsAgent.addEvalFunction( 'electricity_costs', @eval_electricity_costs);
emsAgent.addEvalFunction( 'monetary_costs', @eval_mon_costs);
emsAgent.addEvalFunction( 'E_min', eval_const(14.7));
emsAgent.addEvalFunction( 'E_max', eval_const(83.3));

%% Simulation instance
sim = Simulation( 'building_control_example', T_sim );
sim.config.livePlot = false; % to supress live plotting
sim.config.doCopyCallingScript = false; % Otherwise, this script is copied to results-directory
sim.config.storePlots = false;
sim.config.storeResults = false;

sim.addAgent(emsAgent);

% add figures
figures_all
figures_pareto

tic;
sim.runSimulation();
toc;
