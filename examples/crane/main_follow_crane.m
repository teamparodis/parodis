% clear everything, especially yalmip variables
clear all;
close all;
yalmip('clear');

%% prediction horizon
T_s = 0.25;
N_pred = 20;
T_hor = repmat(T_s, 1, N_pred);

%% leader
model_leader = createModel( @model_crane_linear, T_hor );

controller_leader = SymbolicController();

controller_leader.addBoxConstraint("x", 1, -5, 5);
controller_leader.addBoxConstraint("x", 3, -5*pi/180, 5*pi/180);
controller_leader.addBoxConstraint("u", 1, -2500, 2500);

Q = diag([10 1 0 100]);
R = 5e-6;

controller_leader.addCostFunction( 'costs', LQRCostFunction(N_pred, Q, R) );

x0 = [4 0 0 0]';
leader = Agent('leader', model_leader, controller_leader, T_hor, x0);

%% follower
model_follower = createModel( @model_crane_linear, T_hor );

controller_follower = SymbolicController();

controller_follower.addBoxConstraint("x", 1, -5, 5);
controller_follower.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);
controller_follower.addBoxConstraint("u", 1, -2500, 2500);

Q_follower = diag([10 1 0 100]);
R_follower = 5e-6;

% reference signal via parameter
controller_follower.addParam('x_ref', [4 N_pred+1], @param_x_ref);
% additional argument for LQRCostFunction is a reference trajectory or point
controller_follower.addCostFunction( 'costs', LQRCostFunction(N_pred, Q_follower, R_follower, 'x_ref') );

x0 = [-5 0 0 0]';
follower = Agent('follower', model_follower, controller_follower, T_hor, x0);

%% simulation
T_sim = 30;
sim = Simulation('crane-following-example', T_sim);
sim.addAgent(leader);
sim.addAgent(follower);

%% plots
fig = TimeSeries("crane trajectory", 2, 1);
fig.addLine(leader, 'x', 1, 1, {'Position Leader'}, [], {}, {}, 'left');
fig.addLine(follower, 'x', 1, 1, {'Position Follower'}, [], {}, {}, 'left');
fig.addLine(leader, 'x', 3, 1, {'Angle Leader'}, [], {}, {}, 'right');
fig.addLine(follower, 'x', 3, 1, {'Angle Follower'}, [], {}, {}, 'right');
fig.setFixedYLimits(1, [-10*pi/180 10*pi/180], 'right');
fig.setXLabel(1, 'Time in s');
fig.setYLabel(1, 'Position in m');
fig.setYLabel(1, 'Angle in rad', 'right');

fig.addLine(leader, 'u', 1, 2, {'Actor force leader'}, [], {}, {});
fig.addLine(follower, 'u', 1, 2, {'Actor force follower'}, [], {}, {});
fig.setFixedYLimits(1, [-5 5], 'left');
fig.setFixedYLimits(1, [-10*pi/180 10*pi/180], 'right');
fig.setFixedYLimits(2, [-2500 2500], 'left');
fig.setYLabel(2, 'Force in N');
fig.setXLabel(2, 'Time in s');

fig.setFigureOptions({'Position', [440 400 560 420]});
sim.addPlot(fig);

%% run and animate
sim.config.livePlot = false;
sim.config.storePlots = false;
sim.config.storeResults = false;

sim.runSimulation();

%% animate
animate_crane_sim(sim);