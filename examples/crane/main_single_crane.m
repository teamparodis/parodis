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

%controller = ParetoController();
%controller.config.frontDeterminationScheme = 'AWDS';
controller = SymbolicController();

% cat may move 5m around origin
controller.addBoxConstraint("x", 1, -5, 5);
% max angle of rope +/- 10°
controller.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);

% add LQR cost function
Q = diag([30 1 1000 10]);
R = 5e-3;

% controller.addSharedSlack('z', [1 1]);
% controller.addConstraint(@(m,p,s)(s.z == 0))

controller.addCostFunction( 'costs_q', LQRCostFunction(N_pred, Q, R) );
%controller.addCostFunction( 'costs_r', LQRCostFunction(N_pred, zeros(4), R) );

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

% fig2 = ParetoPlot("test", 1, 1);
% fig2.addParetoFront(crane, [1 2], 1, true);
% sim.addPlot(fig2);

sim.config.livePlot = false;
sim.config.storePlots = false;
sim.config.storeResults = false;

sim.runSimulation();

%%
animate_crane_sim(sim);