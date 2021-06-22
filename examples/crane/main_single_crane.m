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
doPareto = 0;
N_S = 1;
model = createModel( @model_crane_linear_dist, T_hor, N_S);

if ~doPareto
    controller = SymbolicController(N_S);
else
    controller = ParetoController();
    controller.config.frontDeterminationScheme = 'AWDS';
    
end

% cat may move 5m around origin
controller.addBoxConstraint("x", 1, -5, 5);
% max angle of rope +/- 10°
controller.addBoxConstraint("x", 3, -10*pi/180, 10*pi/180);

if model.n_d > 0
    rho_air = 1.2;
    v_wind = 10;
    A_c = 4;
    % assume constant wind force/pressure on container
    controller.predDisturbanceSource = (1/2 * rho_air * v_wind^2 * A_c) * ones(1, N_pred);
    % actual force is dependent on relative speed of container
    %controller.realDisturbanceSource = @(~, crane, ~, ~)({ 1/2 * rho_air * (v_wind + crane.history.x(2, end) + 10*crane.history.x(4, end)*cos(crane.history.x(3, end)))^2 * A_c * cos(crane.history.x(3, end)) });
    controller.realDisturbanceSource = @disturbance_wind;
end

if ~doPareto
    % add LQR cost function
    % Q = diag([30 1 1000 10]);
    Q = diag([10 1 0 1]);
    R = 5e-6;
    
    
    controller.addCostFunction( 'costs_q', LQRCostFunction(N_pred, Q, R) );
    
    % controller.addCostFunction( 'costs_q', LQRCostFunction(N_pred, Q, 0) );
    % controller.addCostFunction( 'costs_r', LQRCostFunction(N_pred, zeros(4), R) );
    
else %do Pareto
    controller.addSharedSlack('z', [1 1]);
    controller.addConstraint(@(m,p,s)(s.z == 0))
    
    Q1 = diag([10 1 0 0]);
    Q2 = diag([0 0 0 1]);
    R = 5e-6;
    
    controller.addCostFunction( 'costs_q1', LQRCostFunction(N_pred, Q1, R/2) );
    controller.addCostFunction( 'costs_q2', LQRCostFunction(N_pred, Q2, R/2) ); 
end
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

if doPareto
    fig2 = ParetoPlot("test", 1, 1);
    fig2.addParetoFront(crane, [1 2], 1, true);
    sim.addPlot(fig2);
end

sim.config.livePlot = false;
sim.config.storePlots = false;
sim.config.storeResults = true;

sim.runSimulation();

%%
weights = crane.history.pareto.chosenParameters;
diff = crane.history.pareto.nadirs  - crane.history.pareto.utopias;
weights_corrected = weights./diff;

%%
animate_crane_sim(sim);