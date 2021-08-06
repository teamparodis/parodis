% clear everything, especially yalmip variables
clear all;
close all;
yalmip('clear');

%% prediction horizon
% time step for discretisation
T_s = 0.25; % in seconds

% length of prediction horizon
N_pred = 20; 

% time horizon vector in seconds
T_hor = repmat(T_s, 1, N_pred) ;

%% setup model
% without disturbance
model = createModel( @model_crane_linear, T_hor);
% with wind disturbance on container
%model = createModel( @model_crane_linear_dist, T_hor);

controller = SymbolicController();

% cat may move 5m around origin
controller.addBoxConstraint("x", 1, -5, 5);
% max angle of rope +/- 10°
controller.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);
% max actuator force on cat in N
controller.addBoxConstraint("u", 1, -2500, 2500);

%% disturbances
% if model considers disturbances
if model.n_d > 0
    % for prediction, assume constant wind force/pressure on container
    rho_air = 1.2; v_wind = 20; A_c = 4;
    F_wind = 1/2 * rho_air * v_wind^2 * A_c;
    controller.predDisturbanceSource = repmat(F_wind, 1, N_pred);
    
    % actual force is dependent on relative speed and angle of container
    controller.realDisturbanceSource = @disturbance_wind;
end

%% cost function
Q = diag([10 1 0 100]);
R = 5e-6;

controller.addCostFunction( 'lqr', LQRCostFunction(N_pred, Q, R) );

%% agent
% initial state
x0 = [-3 0 0 0]';
crane = Agent('crane', model, controller, T_hor, x0);

%% simulation
% simulation length
T_sim = 30; % 30 seconds
sim = Simulation('crane-example', T_sim);
sim.addAgent(crane);

%% plots
fig = TimeSeries("crane trajectory", 2, 1);
fig.addLine(crane, 'x', 1, 1, {'Position'}, [], {}, {}, 'left');
fig.addLine(crane, 'x', 3, 1, {'Angle'}, [], {}, {}, 'right');
fig.setFixedYLimits(1, [-5 5], 'left');
fig.setFixedYLimits(1, [-10*pi/180 10*pi/180], 'right');
fig.setXLabel(1, 'Time in s');
fig.setYLabel(1, 'Position in m');
fig.setYLabel(1, 'Angle in rad', 'right');

fig.addLine(crane, 'u', 1, 2, {'Actor force'}, [], {}, {});
fig.setFixedYLimits(2, [-2500 2500], 'left');

fig.setYLabel(2, 'Force in N');
fig.setXLabel(2, 'Time in s');
fig.setFigureOptions({'Position', [440 400 560 420]});
sim.addPlot(fig);

sim.config.livePlot = false;
sim.config.storePlots = false;
sim.config.storeResults = true;
sim.runSimulation();

%% animation
animate_crane_sim(sim);