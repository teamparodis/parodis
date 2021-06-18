%% This is the main script for the single crane example 

% Delete workspace, add paths
clear all;
close all;
yalmip('clear');

% length of prediction horizon
N_pred = 20; 

% time horizon vector in seconds
T_hor = repmat(0.25, 1, N_pred) ;

%% 1) create model 

model = createModel( @model_crane_linear, T_hor, 1 );

controller = SymbolicController();

% cat may move 5m around origin
controller.addBoxConstraint("x", 1, -5, 5);
% max angle of rope +/- 10°
controller.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);

% add LQR cost function
Q = diag([30 1 100 10]);
R = 5;

controller.addCostFunction( 'costs', LQRCostFunction(N_pred, Q, R) );

%% 2) create agent
% initial state
x0 = [-3 0 0 0]';
crane = Agent('crane', model, controller, T_hor, x0);

%% 3) setup simulation
% simulation length
T_sim = 1*60; % 2 minutes
sim = Simulation('crane-example', T_sim);
sim.addAgent(crane);

%% 4) plots
fig = TimeSeries("crane trajectory", 1, 1);
fig.addLine(crane, 'x', 1, 1, {'Position'}, [], {}, {}, 'left');
fig.addLine(crane, 'x', 3, 1, {'Angle'}, [], {}, {}, 'right');
fig.setFixedYLimits(1, [-10*pi/180 10*pi/180], 'right');

sim.addPlot(fig);

sim.config.livePlot = false;

sim.runSimulation();