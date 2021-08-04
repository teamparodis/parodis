function [ode, n_x, n_u, n_d, linearRepresentation] = model_LL( T_s, T_s_all )
%Creates the ODEs for the lower level model
% States: 9 zone temperatures 
% Inputs: Q_heat_1, ..., Q_heat_9, Q_cool_1, ..., Q_cool_9
% Disturbances: theta_a (outside air temperature)
% ( NO Q_other disturbances respected ) 

%% Model constants

DT = T_s/60;
n_zones = 9;


C_th = 10^-3*[230880 476288 214272 103680 330144 330144 99456 2400 4800]; % thermal capacity of temperature zones 
H_air = 10^-3*[3688.424295 9819.744604 3654.827624 2788.84736 4787.074076 6193.049076 3191.137352 25.7808 44.9616]; % heat transfer coefficient from temperature zone to the outside air

beta = 10^-3*[    0   0   0   0   0      0      0   0    0;             % 1
                  0   0   0   0   0      0      0   0   48.4;           % 2
                  0   0   0 345.6 0      0      0   0    0;
                  0   0   0   0   0      0      0   0    0;
                  0   0   0   0   0   1100.48   0  23.4  0;             % 5
                  0   0   0   0   0      0      0   8    0;
                  0   0   0   0   0      0      0   0    0;
                  0   0   0   0   0      0      0   0    0;             % 8
                  0   0   0   0   0      0      0   0    0   ]; % heat transfer coefficient from temperature zone i to temperature zone j and vice versa

% TESTWEISE: BETA = 0 !
% beta = zeros(size(beta)); 
% warning('Nicht vergessen, dass ich beta = 0 gesetzt habe!')

% Mirroring betas + negative sume in diagonal
beta = beta+ beta'; 

for ii = 1:n_zones
    beta_diag(ii) = -sum(beta(ii,:));   % since beta_{j,j} is 0 anyway
end

for ii = 1 : length(C_th)
    C_th_matrix(ii,1:n_zones) = C_th(ii);
end

%% System Matrix A

% System matrix just for temperature zones:
A_cont = (-diag(H_air) + beta + diag(beta_diag)).*(1./C_th_matrix);

%%  Input Matrix B
% First: temperature part
B_cont = [diag(1./C_th), diag(1./C_th)];

%% Disturbance Matrix S
S_cont =  (H_air./C_th)';


%% Discretize System(s)
Adis = expm(A_cont*DT); 

h1 = @(tau) expm(A_cont*(DT-tau))*B_cont;
Bdis = integral(h1, 0, DT, 'ArrayValued', true);

h2 = @(tau) expm(A_cont*(DT-tau))*S_cont;
Sdis = integral(h2, 0, DT, 'ArrayValued', true);

ode = @(x, u, d)( Adis*x +  Bdis*u +  Sdis * d);

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
