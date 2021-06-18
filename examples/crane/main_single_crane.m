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
sim.config.storePlots = false;
sim.config.storeResults = false;

sim.runSimulation();

%%
animate(crane);

function animate(crane)
    fig = figure;
    ax = axes;
    l = 10;
    dt = 0.25;
    
    for i=1:length(crane.history.x)
        x = crane.history.x(:, i);
        cat_pos = [x(1) 0];
        container_pos = l * [sin(x(3)) -cos(x(3))] + cat_pos;
        cla(ax);
        hold on
        rectangle('Position', [cat_pos(1)-0.25 cat_pos(2) 0.5 0.25]);
        line([container_pos(1) cat_pos(1)], [container_pos(2) cat_pos(2)]);
        line([-5 5], [0 0], 'Color', [0 0 0])
        
        w = 0.6;
        h = 0.3;
        
        poly_x = [0 0 w w];
        poly_y = [0 h h 0];
        container_poly = translate( rotate( polyshape(poly_x, poly_y), x(3)*180/pi ), container_pos - [w h]/2);
        %plot(container_poly)
        
        rectangle('Position', [container_pos(1)-0.3 container_pos(2)-0.15 0.6 0.3], 'FaceColor', [1 1 1]);
        xlim([-5 5]);
        ylim([-11 1]);
        drawnow
        %pause(dt);
    end
    disp("done");
end