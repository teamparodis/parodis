function [ode, n_x, n_u, n_d, linearRepresentation] = model_crane_linear( T_s, T_s_all )

% constant values, all in SI units
mc = 4000; % mass of crane in kg
mk = 1000; % mass of cat in kg
l = 10; % length of rope in m
g = 9.81; % gravity

withDisturbanceOnCrane = 0; 

% continuos (linearized model) 

Acont = [0 1 0 0; 
         0 0 mc*g/mk 0; 
         0 0 0 1; 
         0 0 -(mk+mc)*g/(mk*l) 0]; 
     
Bcont = [0; 1; 0; -1/l]; 

if withDisturbanceOnCrane
    Scont = Bcont;
else
    Scont = [];
end


% Discretize System(s)

DT = T_s;

Adis = expm(Acont*DT); 

h1 = @(tau) expm(Acont*(DT-tau))*Bcont;
Bdis = integral(h1, 0, DT, 'ArrayValued', true);

if withDisturbanceOnCrane
    h2 = @(tau) expm(Acont*(DT-tau))*Scont;
    Sdis = integral(h2, 0, DT, 'ArrayValued', true);
    ode = @(x, u, d)( Adis*x +  Bdis*u +  Sdis*d);
else
    Sdis = [];
    ode = @(x, u, d)( Adis*x +  Bdis*u );
end


% additional linear representation
linearRepresentation = struct;
linearRepresentation.A = Adis;
linearRepresentation.B = Bdis;
linearRepresentation.S = Sdis;


%% set output parameters
n_x = length(Adis);
n_u = size(Bdis, 2);
n_d = size(Sdis, 2);

end