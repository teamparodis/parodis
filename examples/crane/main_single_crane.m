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
% without disturbance
model = createModel( @model_crane_linear, T_hor, 1);
% with wind disturbance on container
%model = createModel( @model_crane_linear_dist, T_hor, 1);

controller = SymbolicController();

% cat may move 5m around origin
controller.addBoxConstraint("x", 1, -5, 5);
% max angle of rope +/- 10°
controller.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);
% max actuator force on cat in Newton
controller.addBoxConstraint("u", 1, -2500, 2500);

if model.n_d > 0
    rho_air = 1.2; v_wind = 20; A_c = 4;
    F_wind = 1/2 * rho_air * v_wind^2 * A_c;
    % assume constant wind force/pressure on container
    controller.predDisturbanceSource = repmat(F_wind, 1, N_pred);
    
    % actual force is dependent on relative speed of container
    %controller.realDisturbanceSource = @(~, crane, ~, ~)({ 1/2 * rho_air * (v_wind + crane.history.x(2, end) + 10*crane.history.x(4, end)*cos(crane.history.x(3, end)))^2 * A_c * cos(crane.history.x(3, end)) });
    controller.realDisturbanceSource = @disturbance_wind;
end

Q = diag([10 1 0 100]);
R = 5e-6;

controller.addCostFunction( 'costs_q', LQRCostFunction(N_pred, Q, R) );

%% 2) create agent
% initial state
x0 = [-3 0 0 0]';
crane = Agent('crane', model, controller, T_hor, x0);

%% 3) setup simulation
% simulation length
T_sim = 30; % 20 seconds
sim = Simulation('crane-example', T_sim);
sim.addAgent(crane);

%% 4) plots
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

%%
animate_crane_sim(sim);