function [ode, n_x, n_u, n_d, linearRepresentation] = model_crane_linear( T_s, T_s_all )
% x = [cat position, cat velocity, rope angle, rope angular velocity]
% u = actor force on cat

% constant values, all in SI units
mc = 4000; % mass of crane in kg
mk = 1000; % mass of cat in kg
l = 10; % length of rope in m
g = 9.81; % gravity

% continuos (linearized model) 

Acont = [0 1 0 0; 
         0 0 mc*g/mk 0; 
         0 0 0 1; 
         0 0 -(mk+mc)*g/(mk*l) 0]; 
     
Bcont = [0; 1; 0; -1/l] / mk; 

% Discretize System(s)

DT = T_s;

Adis = expm(Acont*DT); 

h1 = @(tau) expm(Acont*(DT-tau))*Bcont;
Bdis = integral(h1, 0, DT, 'ArrayValued', true);

Sdis = [];

%% PARODIS
ode = @(x, u, d)( Adis*x +  Bdis*u );
n_x = length(Adis); % number of state
n_u = size(Bdis, 2); % number of inputs
n_d = size(Sdis, 2); % number of disturbances

% additional linear representation, speeds things up internally
linearRepresentation = struct;
linearRepresentation.A = Adis;
linearRepresentation.B = Bdis;
linearRepresentation.S = Sdis;

end