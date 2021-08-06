function model = createModel(model_fun, T_s, numScenarios, implicitPrediction, controller)
% model = createModel creates the model struct
%
%   model_fun           function handle returning [ode, n_x, n_u, n_d] for a given sampling time
%   T_s                 vector with sampling times over horizon, lengcreateModelth(T_s) is length of horizon
%   numScenarios        number of scenarios to evaluate
%   implicitPrediction  if true, state predictions are implicit and defined by constraints x(k+1) == f(x(k), u(k), d(k))
%                       otherwise x(k+1) is an explicit expression of u(k) and d(k)

    if nargin < 3
        numScenarios = 1;
    end

    if nargin < 4
        implicitPrediction = false;
    end

    if nargin < 5
        controller = [];
    end
    
    % get parameters for presetting sdpvars
    [ode_k, n_x, n_u, n_d] = model_fun(T_s(1));
    
    % preset sdpvars for all expressions
    model.x0 = sdpvar(n_x, 1);
    model.u = sdpvar(n_u, length(T_s), 'full');
    model.n_x = n_x;
    model.n_u = n_u;
    model.n_d = n_d;
    model.model_fun = model_fun;
    model.odes = {};
    model.ode0 = ode_k;
    model.implicitPrediction = implicitPrediction;
    model.parameterVariant = nargin(ode_k) > 3;
    
    % test is model function returns x_flat
    % must take vector of all T_s as input
    if nargin( model_fun ) == 2
        % if model fun returns function handle for x_flat, store, otherwise continue as usual
        try
            [~, ~, ~, ~, x_flat] = model_fun(T_s(1), T_s);
            if isa(x_flat, 'function_handle')
                model.x_flat = x_flat;
            end
        catch
            
        end
    end

    % test if system supports fast linear representation, i.e. x(k+1) = Ax+Bu+Sd
    try
        [~, ~, ~, ~, linRep] = model_fun(T_s(1));
        hasLinearRepresentation = isa(linRep, 'struct');
        flrMatrices = linRep;
    catch E
        hasLinearRepresentation = false;
    end
    
    N_pred = length(T_s);

    for s=1:numScenarios
        % preset [x(0|k) ... x(n|k)] as sdpvar, but keep x(0|k) = x0
        model.x{s} = [model.x0 sdpvar(n_x, length(T_s), 'full')];
        model.d{s} = sdpvar(n_d, length(T_s), 'full');
    end
    
    for k=1:N_pred
        % only refetch ODE if T_s differs
        if k > 1 && T_s(k-1) ~= T_s(k)
            % if linear representation is supported, store matrices
            if hasLinearRepresentation
                [ode_k, ~, ~, ~, linRep] = model_fun(T_s(k));
            else
                [ode_k, ~, ~, ~] = model_fun(T_s(k));
            end
        end
        
        model.odes{end+1} = ode_k;
        
        if hasLinearRepresentation
            flrMatrices(k) = linRep;
        end
        
        % only define x(k+1) directly in terms of u and d if explicit
        % prediction is used
        if ~implicitPrediction
            for s=1:numScenarios
                if model.parameterVariant
                    model.x{s}(:, k+1) = ode_k(model.x{s}(:, k), model.u(:, k), model.d{s}(:, k), k, extractScenario(controller.paramSyms, s));
                else
                    model.x{s}(:, k+1) = ode_k(model.x{s}(:, k), model.u(:, k), model.d{s}(:, k));
                end
            end
        end
    end
    
    % if system has linear representation, attempt to build x_flat for the linear case
    % i.e. a function that efficiently calculates the entire trajectory at once
    if hasLinearRepresentation
        % if A, B or S is parameter variant, we can't prebuild x_flat yet
        if isa(flrMatrices(1).A, 'function_handle') || isa(flrMatrices(1).B, 'function_handle') || isa(flrMatrices(1).S, 'function_handle')
            model.flrMatrices = flrMatrices;
        else
            A_tilde = [eye(n_x); zeros(n_x*N_pred, n_x)];
            B_tilde = zeros(n_x*(N_pred+1), N_pred*n_u);
            S_tilde = zeros(n_x*(N_pred+1), N_pred*n_d);

            for n = 1:N_pred
                A = flrMatrices(n).A;
                B = flrMatrices(n).B;
                S = flrMatrices(n).S;


                A_tilde( n*n_x+1:(n+1)*n_x, : ) = A * A_tilde( (n-1)*n_x+1:n*n_x, : );
                B_tilde( n*n_x+1:(n+1)*n_x, : ) = A * B_tilde( (n-1)*n_x+1:n*n_x, : );
                B_tilde( n*n_x+1:(n+1)*n_x, (n-1)*n_u+1:n*n_u ) = B;

                S_tilde( n*n_x+1:(n+1)*n_x, : ) = A * S_tilde( (n-1)*n_x+1:n*n_x, : );
                S_tilde( n*n_x+1:(n+1)*n_x, (n-1)*n_d+1:n*n_d ) = S;
            end

            model.x_flat = @(x0, uPred_flat, dPred_flat, ~)( A_tilde *  x0 + B_tilde * uPred_flat + S_tilde * dPred_flat );
        end
    end
    
    
end