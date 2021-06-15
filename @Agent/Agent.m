classdef Agent < handle
    properties
        % controller object used for calculating optimal inputs
        controller
        
        % model struct containing symbolic model expressions
        model
        
        % struct containing realised trajectories, parameters
        history
        
        % struct containing predicted trajectories, parameters
        virtualHistory
        
        % struct containing configuration parameters
        config
       
        % struct containing current simulation status (predictions, param values etc.)
        status
        
        % the agent's name
        name
        
        % callbacks
        callbackMeasureState
        
        % callback for setting data after agent's negotiation step
        callbackNegotiation
        
        % callback for performing calculations/communications right after optimal input is retrieved
        callbackPostOptimization
        
        % simulation object that created the agent
        simulation
        
        % timestep seeds generated for each timestep for better reproduceability
        timestepSeeds
    end
    
    properties (Access = protected)
        logFileHandle
    end
    
    properties (SetAccess = protected)
        doPareto
        previousStatus
        uPrev_0
    end
    
    properties (Hidden = true)
        difHistory
    end
    
    methods
        function obj = Agent(name, model, controller, T_s, x0, uPrev_0)
            obj.name = name;
            obj.model = model;
            obj.controller = controller;
            
            % set default config
            obj.config = Agent.getDefaultConfig();
            obj.config.T_s = T_s;
            
            % initialise history struct
            obj.history = struct;
            obj.history.x = x0;
            obj.history.u = [];
            obj.history.d = [];
            obj.history.evalValues = struct;
            obj.history.simulationTime = [0];
            obj.history.costs = struct;
            
            % optional initial input to be assumed for constraint on u(0|k) = u(0|k) - u(k-1)
            if nargin < 6
                uPrev_0 = zeros(model.n_u, 1);
            end
            obj.uPrev_0 = uPrev_0;
            
            % call each cost function and retrieve horizon
            costNames = fieldnames(controller.costFunctionIndexes);
            for idx = 1:length(costNames)
                obj.history.costs.(costNames{idx}) = [];
            end
            
            obj.virtualHistory = obj.history;
            obj.difHistory = obj.history;
            
            obj.history.pareto = struct;
            obj.history.pareto.fronts = {};
            obj.history.pareto.chosenSolutions = [];
            obj.history.pareto.paretoParameters = {};
            obj.history.pareto.utopias = [];
            obj.history.pareto.nadirs = [];
            obj.history.pareto.chosenParameters = [];
            obj.history.pareto.frontDeterminationScheme = [];
            
            % initialise status struct
            obj.status = struct;
            obj.status.k = 0;
            obj.status.xPred = [];
            obj.status.uPred = [];
            obj.status.dPred = [];
            obj.status.evalPred = struct;
            obj.status.paramValues = [];
            obj.status.chosenParameters = [];
            obj.status.horizonTime = [];
            obj.status.costsPred = struct;
            obj.status.slackVariables = struct;
            obj.status.solverCode = 0;
            
            obj.doPareto = strcmp(controller.type, 'pareto');
            
            obj.status.pareto = struct;
            obj.status.chosenParameters = [];
            
            % initially, previous state should conform to isempty
            obj.previousStatus = [];
            
            if length(x0) ~= model.n_x
                warning("PARODIS Agent:constructor The provided value for x0 is malformed (%s)", name);
            end
            
            if length(uPrev_0) ~= model.n_u
                warning("PARODIS Agent:constructor The provided value for u0 is malformed (%s)", name);   
            end
            
        end
        
        function initialise(this)
            % initialise    Called by simulation right after addAgent
            %               Creates the sdpsettings object according to
            %               agent configuration and calls
            %               controller.compile with it
            
            if this.config.debugMode
                options = {'solver', this.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
            else
                options = {'solver', this.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
            end
            
            options = [options, this.config.solverOptions];
            yalmipOptions = sdpsettings( options{:} );
            
            this.log("start compiling controller");
            this.controller.compile(this.model, this, yalmipOptions);
            this.log("controller compiled.");
        end
        
        function addEvalFunction(this, name, function_handle, scenarioDependent)
            if nargin < 4
                scenarioDependent = false; 
            end
            
            this.config.evalFuns.(name) = {function_handle, scenarioDependent};
            this.history.evalValues.(name) = [];
            this.virtualHistory.evalValues.(name) = [];
            this.status.evalPred.(name) = repmat( {NaN(size(this.config.T_s))}, this.controller.numScenarios, 1);
        end
        
        function doNegotiation(this, externalData)
            % doNegotation    Executes a negotiation step for the agent
            
            if nargin < 2
                externalData = [];
            end
            
            this.updateRng(); 
            this.measureState( externalData );
            this.status.dPred = this.getDisturbance( externalData );
            this.setParameterValues();
            
            this.status.uPred = this.getOptimalTrajectory();
            
            if ~isempty(this.callbackPostOptimization)
                this.callbackPostOptimization(this, this.simulation)
            end
            
            if isfield(this.status, 'solverCode') && this.status.solverCode ~= 0
                if strcmpi( this.config.pauseOnError, 'on') || (this.config.pauseOnError == this.status.solverCode)
                    message = sprintf("Paused execution, solver returned code %i: %s", this.status.solverCode, yalmiperror(this.status.solverCode));
                    this.log( message, true );

                    pause;
                end
            end
            
            if ~isempty(this.callbackNegotiation)
               this.callbackNegotiation(this, this.simulation); 
            end
        end
        
        function doStep(this, externalData)
            % doStep    Executes a simulation step for the agent
            
            if nargin < 2
                externalData = [];
            end
            
            this.updateRng();
            this.measureState( externalData );
            this.status.dPred = this.getDisturbance( externalData );
            this.setParameterValues();
            
            % if negotiation was executed and optimal input generated during negotiation shall be reused
            % we shall no re-execture getOptimalTrajectory and simply use status.uPred directly
            if ~this.config.reuseNegotiationResult || isempty(this.status.uPred)
                this.status.uPred = this.getOptimalTrajectory();

                if ~isempty(this.callbackPostOptimization)
                    this.callbackPostOptimization(this, this.simulation)
                end
            end
            
            this.updatePredictions();
            
            this.simulation.updatePlots(this);
            
            if isfield(this.status, 'solverCode') && this.status.solverCode ~= 0
                if strcmpi( this.config.pauseOnError, 'on') || (this.config.pauseOnError == this.status.solverCode)
                    message = sprintf("Paused execution, solver returned code %i: %s", this.status.solverCode, yalmiperror(this.status.solverCode));
                    this.log( message, true );

                    pause;
                end
            end
            
            this.updateHistory( externalData );
            
            this.previousStatus = this.status;
            this.clearStatus();
            
            this.status.k = this.status.k + 1;
            
        end
        
        function u = getOptimalTrajectory(this)
            % getOptimalTrajectory  Retrieves an optimal input, either from
            %                       pareto evaluation or directly from controller, stores input
            %                       in current status
            
            x0 = this.history.x(:, end);
            if size(this.history.u, 2) > 0 && ~any(isnan(this.history.u(:, end)))
                uPrev = this.history.u(:, end);
            else
                uPrev = this.uPrev_0;
            end
                
            if ~this.doPareto
                [u, slacks, code] = this.controller.getInput(x0, uPrev, this);
            else
                [u, slacks, code, paretoResults] = this.controller.getInput(x0, uPrev, this);
                
                % should include fields: chosenParameters, parameters, front, utopia, nadir
                this.status.pareto = paretoResults;
            end
            
            this.status.solverCode = code;
            this.status.slackVariables = slacks;
        end
        
        function updatePredictions(this)
            % updatePredictions  Sets state and eval predictions according
            %                    to currently available input and state predictions from
            %                    current agent state
            
            this.status.xPred = this.predictTrajectory(this.status.uPred, this.status.dPred, this.history.x(:, end));
            
            % call each eval function with pred = true and store result
            this.status.evalPred = this.evaluateEvalFunctions(true);
            
            % call each cost function and retrieve horizon
            this.status.costsPred = this.evaluateCostFunctions(this.status.xPred, this.status.uPred, this.status.dPred);
        end
        
        function updateHistory(this, externalData)
            % updateHistory     Retrieves realised disturbance and applies
            %                   it together with available u(0|k) to the ODE to get the next
            %                   timestep, updates history accordingly
            
            if nargin < 2
                externalData = [];
            end
            
            T_current = this.history.simulationTime(end);
            x0 = this.history.x(:, end);
            u0 = this.status.uPred(:, 1);
            d0_pred = this.status.dPred{1}(:, 1);
            d0_real = this.controller.getRealDisturbance(T_current, this, this.simulation.agents);
            
            % update real history
            if ~isempty(externalData)
                d0_real = externalData.disturbances{1}(:, 1);
            end
            
            % TODO: maybe change this to take averaged scenario?
            x1_pred = this.status.xPred{1}(:, 2);
            
            if this.model.parameterVariant
                x1_real = this.model.ode0(x0, u0, d0_real, 1, extractScenario(this.status.paramValues, 1));
            else
                x1_real = this.model.ode0(x0, u0, d0_real);
            end
            
            this.history.x(:, end+1) = x1_real;
            this.history.u(:, end+1) = u0;
            this.history.d(:, end+1) = d0_real;
            
            this.virtualHistory.x(:, end+1) = x1_pred;
            this.virtualHistory.u(:, end+1) = u0;
            this.virtualHistory.d(:, end+1) = d0_pred;
            
            if this.doPareto
                this.history.pareto.fronts{end+1} = this.status.pareto.front;
                this.history.pareto.chosenSolutions(end+1, :) = this.status.pareto.chosenSolution;
                this.history.pareto.paretoParameters{end+1} = this.status.pareto.paretoParameters;
                this.history.pareto.utopias(end+1, :) = this.status.pareto.utopia;
                this.history.pareto.nadirs(end+1, :) = this.status.pareto.nadir;
                this.history.pareto.chosenParameters(end+1, :) = this.status.pareto.chosenParameters;
                this.history.pareto.frontDeterminationScheme{end+1} = this.controller.config.frontDeterminationScheme;
            end
            
            % to calculate predicted eval results, we call evaluateEvalFunctions with pred = true
            % and then take the first entry for the first scenario
            % TODO: maybe change this to take averaged scenario?
            evalValues_pred = this.evaluateEvalFunctions(true);
            this.virtualHistory.evalValues = mapToStruct(this.virtualHistory.evalValues, @(s, field)( [s.(field) evalValues_pred.(field){1}(1)] ) );
            
            evalValues_real = this.evaluateEvalFunctions(false);
            this.history.evalValues = mapToStruct(this.history.evalValues, @(s, field)( [s.(field) evalValues_real.(field)] ) );
            
            % call each cost function and retrieve horizon
            d = this.status.dPred;
            for s=1:length(this.status.dPred)
                d{s}(:, 1) = d0_real;
            end
            
            costNames = fieldnames(this.controller.costFunctionIndexes);
            if length(costNames) > 0
                % only recalculate trajectory is disturbance wasn't measured
                % otherwise, xPred in status is already correct
                if this.config.disturbanceMeasured
                    xPred = this.status.xPred;
                else
                    xPred = this.predictTrajectory( this.status.uPred, d, x0 );
                end
                
                costValues_real = this.evaluateCostFunctions(xPred, this.status.uPred, d);
                this.history.costs = mapToStruct(this.history.costs, @(s, field)( [s.(field) costValues_real.(field)(1)] ) );
                
                costValues_pred = this.evaluateCostFunctions(this.status.xPred, this.status.uPred, this.status.dPred);
                this.virtualHistory.costs = mapToStruct(this.virtualHistory.costs, @(s, field)( [s.(field) costValues_pred.(field)(1)] ) );
            end
            
            this.history.simulationTime(:, end+1) = T_current + this.config.T_s(1);
            this.virtualHistory.simulationTime(:, end+1) = T_current + this.config.T_s(1);
        end
        
        function evalValues = evaluateEvalFunctions(this, predict)
            evalValues = struct;
            
            evalNames = fieldnames(this.config.evalFuns);
            for idx = 1:length(evalNames)
                evalName = evalNames{idx};
                evalFun = this.config.evalFuns.(evalName){1};
                scenarioDependent = this.config.evalFuns.(evalName){2};
                if predict
                    if scenarioDependent
                        this.status.evalPred.(evalName) = cell(this.controller.numScenarios, 1);

                        for s=1:this.controller.numScenarios
                            data = evalFun(this, this.simulation, true, s);
                            evalValues.(evalName){s} = data;
                        end
                    else
                        data = evalFun(this, this.simulation, true, 1);
                        evalValues.(evalName) = repmat({data}, this.controller.numScenarios, 1);
                    end
                else
                    data = evalFun(this, this.simulation, predict, []);
                    evalValues.(evalName) = data;
                end
            end
            
        end
        
        function costValues = evaluateCostFunctions(this, xPred, uPred, dPred)
            costValues = struct;
            
            costNames = fieldnames(this.controller.costFunctionIndexes);
            for idx = 1:length(costNames)
                costName = costNames{idx};
                costIdx = this.controller.costFunctionIndexes.(costName);
                costFunction = this.controller.costFunctions{costIdx};
                
                data = costFunction.evaluateHorizon( xPred, uPred, dPred, ... 
                                                     this.status.paramValues, this.status.slackVariables, this.config.T_s );
                costValues.(costName) = data;
            end
        end
        
        function xPred = predictTrajectory(this, uPred, dPred, x0)
            % xPred = predictTrajectory Calculates the predicted
            % state trajectories for a given realisation of uPred and a set
            % of dPred and an initial x0
            % uPred     predicted input u
            % dPred     predicted scenarios of disturbance d
            % x0        initial state x0
            
            N_S = length(this.model.x);
            N_pred = length(this.config.T_s);
            if ~isa(uPred, 'sdpvar')
                xPred = repmat( { NaN(this.model.n_x, N_pred +1 ) }, N_S, 1 );
            else
                xPred = this.model.x;
            end
            
            odes = this.model.odes;
            parameterVariant = this.model.parameterVariant;
            
            n_x = this.model.n_x;
            n_u = this.model.n_u;
            n_d = this.model.n_d;

            uPred_flat = uPred(:);
            
            for s = 1:N_S
                
                dPred_flat = dPred{s}(:);
            
                if ~isa(uPred, 'sdpvar')
                    xPred{s}(:, 1) = x0;
                end
                
                params = extractScenario( this.status.paramValues, s );
                
                % if model has a x_flat function, that returns state trajectory in one call, use that one
                if isfield(this.model, 'x_flat')
                    x_flat = this.model.x_flat(x0, uPred_flat, dPred_flat, params);
                    xPred{s} = reshape(x_flat, n_x, N_pred+1);
                    
                % otherwise, if fast linear representation is possible, i.e. matrices A, B, S available for each T_s in
                % horizon, calculate trajectory using these matrices
                elseif isfield(this.model, 'flrMatrices')
                    flrMatrices = this.model.flrMatrices;
                    
                    A_tilde = [eye(n_x); zeros(n_x*N_pred, n_x)];
                    B_tilde = zeros(n_x*(N_pred+1), N_pred*n_u);
                    S_tilde = zeros(n_x*(N_pred+1), N_pred*n_d);
                    
                    for n = 1:N_pred
                        A = flrMatrices(n).A;
                        B = flrMatrices(n).B;
                        S = flrMatrices(n).S;

                        if isa(A, 'function_handle')
                            A = A(n, params);
                        end
                        if isa(B, 'function_handle')
                            B = B(n, params);
                        end

                        if isa(S, 'function_handle')
                            S = S(n, params);
                        end

                        A_tilde( n*n_x+1:(n+1)*n_x, : ) = A * A_tilde( (n-1)*n_x+1:n*n_x, : );
                        B_tilde( n*n_x+1:(n+1)*n_x, : ) = A * B_tilde( (n-1)*n_x+1:n*n_x, : );
                        B_tilde( n*n_x+1:(n+1)*n_x, (n-1)*n_u+1:n*n_u ) = B;

                        S_tilde( n*n_x+1:(n+1)*n_x, : ) = A * S_tilde( (n-1)*n_x+1:n*n_x, : );
                        S_tilde( n*n_x+1:(n+1)*n_x, (n-1)*n_d+1:n*n_d ) = S;
                    end

                    x_flat = A_tilde *  x0 + B_tilde * uPred_flat + S_tilde * dPred_flat;
                    xPred{s} = reshape(x_flat, n_x, N_pred+1);
                    
                % otherwise, calculate trajectory manually using ODEs
                else
                    % calculate trajectory by numerically evaluating ODE for each step in horizon
                    % this is still slow, but significantly faster than
                    % using replace() on the symbolic expressions for x
                    for n = 1:N_pred
                        if n == 1
                            x_n = x0;
                        else
                            x_n = xPred{s}(:, n);
                        end
                        
                        if parameterVariant
                            xPred{s}(:, n+1) =  odes{n}( x_n, uPred(:, n), dPred{s}(:, n), n, params );
                        else
                            xPred{s}(:, n+1) =  odes{n}( x_n, uPred(:, n), dPred{s}(:, n) );
                        end
                    end
                    
                    if isa(uPred, 'sdpvar')
                        xPred{s}(:, 1) = replace(xPred{s}(:, 1), xPred{s}(:, 1), x0);
                    end
                end
                
            end
        end
        
        function dPred = getDisturbance(this, externalData)
            % dPred = getDisturbance  Retrieves the disturbance prediction for the current timestep
            %                         Will replace the first time step in
            %                         the disturbance with the real
            %                         disturbance if disturbanceMeasures = true
            
            if nargin < 2
                externalData = [];
            end
            
            numScenarios = length(this.model.x);
            N_pred = length(this.status.horizonTime);
            
            if this.model.n_d == 0
                
                dPred = repmat( {zeros(0, N_pred)}, numScenarios, 1);
                
                return;
            end
            
            dPred = this.controller.getPredDisturbance(this.status.horizonTime, this, this.simulation.agents);
            
            if this.config.disturbanceMeasured
               dReal = this.controller.getRealDisturbance(this.status.horizonTime(1), this, this.simulation.agents);
               
               for s=1:numScenarios
                   dPred{s}(:, 1) = dReal;
               end
            end
            
            if ~isempty(externalData)
                % replace only those disturbance predictions that are actually provided externally
                % those that shall not be replaced/are not provided are marked with NaN in externalData.disturbances{s}
                
                dim = size( externalData.disturbances{1} );
                
                % there must be n_d rows and either 1 or N_pred columns
                if dim(1) ~= this.model.n_d || ( dim(2) ~= 1 && dim(2) ~= N_pred )
                    warning( "PARODIS Agent:getDisturbance external disturbance data appears to be malformed, must be n_d x 1 or n_d x N_pred (%s)", this.name );
                end
                    
                for s=1:numScenarios
                    idx = ~isnan( externalData.disturbances{s} );
                    dPred{s}( idx ) = externalData.disturbances{s}( idx );
                end
                
            end
        end
        
        function setParameterValues(this)
            % setParameterValues    Retrieves values for all parameters and stores them in the status
            
            paramNames = fieldnames(this.controller.paramConfig);
            paramValues = struct;
            for idx = 1:length(paramNames)
                paramName = paramNames{idx};
                [dimensions, source, ~] = this.controller.paramConfig.(paramName){:};
                paramValues.(paramName) = this.simulation.sourceManager.getDataFromSource( ...
                    this.status.horizonTime, source, this, this.simulation.agents, this.controller.numScenarios ...
                );
                
                if( ~isequal(dimensions, size( paramValues.(paramName){1} )))
                    warning("PARODIS Agent:setParamValues dimensions of parameter source for " + paramName + " do not agree (%s)", this.name);
                end
            end
            
            this.status.paramValues = paramValues;
        end
        
        function measureState(this, externalData)
            % measureState  Sets the current prediction horizon and 
            %               Calls measurement callback, if defined
            
            if nargin < 2
                externalData = [];
            end
            
            % build time vector from current simulation time over horizon
            T_current = this.history.simulationTime(end);
            T_horizon = T_current + cumsum([0 this.config.T_s(1:end-1)]);
            
            this.status.horizonTime = T_horizon;
            
            % if external data was provided
            if ~isempty(externalData)
               this.history.x(:, end) = externalData.state;
               
            % call callback if it's set
            elseif ~isempty(this.callbackMeasureState) && T_current > 0
                this.callbackMeasureState(this, this.simulation);
            % quit if state isn't measured at all
            else
                return;
            end
            
            % if we are in step 0, do not re-evaluate
            % or, if we have restored the simulation at some point k != 0 and thus previousStatus is empty
            if T_current == 0 || isempty(this.previousStatus)
                return;
            end
            
            currentStatus = this.status;
            
            % restore previous status to re-evaluate eval functions for the modified state
            this.status = this.previousStatus;
            
            % re-evaluate eval functions for the modified state
            evalValues = this.evaluateEvalFunctions(false);
            this.history.evalValues = mapToStruct(this.history.evalValues, @(s, field)( [s.(field)(:, 1:end-1) evalValues.(field)] ) );

            % re-evaluate cost horizon for the modified state
            xPred = this.status.xPred;
            dPred = this.status.dPred;
            uPred = this.status.uPred;
            
            % set actually realised state, disturbances and inputs as first value in horizon
            % since we only use the first entry of evaluateHorizon, we don't need to adjust the rest of the horizon
            for s=1:length(xPred)
                xPred{s}(:, 1) = this.history.x(:, end);
                dPred{s}(:, 1) = this.history.d(:, end);
            end
            uPred(:, 1) = this.history.u(:, end);
            
            costValues = this.evaluateCostFunctions(xPred, uPred, dPred);
            this.history.costs = mapToStruct(this.history.costs, @(s, field)( [s.(field)(:, 1:end-1) costValues.(field)(1) ] ) );
            
            % clear status again and set time step back to corresponding step
            this.status = currentStatus;
        end
        
        
        function clearHistory( this, after_k )
            if size(this.history.x, 2) <= after_k+1
                return;
            end
            
            this.history.x(:, after_k+2:end) = [];
            this.history.u(:, after_k+1:end) = [];
            this.history.d(:, after_k+1:end) = [];

            this.history.simulationTime(after_k+2:end) = [];

            this.virtualHistory.x(:, after_k+2:end) = [];
            this.virtualHistory.u(:, after_k+1:end) = [];
            this.virtualHistory.d(:, after_k+1:end) = [];
            
            this.virtualHistory.simulationTime(after_k+2:end) = [];
            
            evalNames = fieldnames(this.history.evalValues);
            for idx = 1:length(evalNames)
                this.history.evalValues.(evalNames{idx})(:, after_k+1:end) = [];
                this.virtualHistory.evalValues.(evalNames{idx})(:, after_k+1:end) = [];
            end
            
            costNames = fieldnames(this.controller.costFunctionIndexes);
            for idx = 1:length(costNames)
                this.history.costs.(costNames{idx})(:, after_k+1:end) = [];
                this.virtualHistory.costs.(costNames{idx})(:, after_k+1:end) = [];
            end
            
            
            if this.doPareto
                this.history.pareto.fronts(after_k+1:end) = []; % TODO: {}?
                this.history.pareto.paretoParameters(after_k+1:end) = [];
                this.history.pareto.utopias(after_k+1:end, :) = [];
                this.history.pareto.nadirs(after_k+1:end, :) = [];
                this.history.pareto.chosenParameters(after_k+1:end, :) = [];
            end
        end
        
        function clearStatus(this, clearPreviousStatus)
            % clearStatus  Clears all fields in the status struct except for the current simulation step k
            % resetPreviousStatus    If true, previousStatus will be cleared as well
            if nargin < 2
                clearPreviousStatus = false;
            end
            
            this.status.xPred = [];
            this.status.uPred = [];
            this.status.dPred = [];
            this.status.evalPred = struct;
            this.status.paramValues = struct;
            this.status.pareto = struct;
            
            if clearPreviousStatus
                this.previousStatus = [];
            end
        end
        
        function storeHistory(this)
            this.log("storing history");
            
            directory = this.simulation.resultDirectory + filesep + this.name + filesep;
            directory_virtual = directory + "virtual" + filesep;
            
            timeVector = this.history.simulationTime;
            
            if ~exist(this.simulation.resultDirectory, 'dir')
               mkdir(this.simulation.resultDirectory)
            end
            
            if ~exist(directory, 'dir')
               mkdir(directory)
            end
            
            if ~exist(directory_virtual, 'dir')
               mkdir(directory_virtual)
            end
            
            csvwrite(directory + "x.csv", [timeVector' this.history.x']);
            csvwrite(directory + "u.csv", [timeVector(1:end-1)' this.history.u']);
            csvwrite(directory + "d.csv", [timeVector(1:end-1)' this.history.d']);
            
            csvwrite(directory_virtual + "x.csv", [timeVector' this.virtualHistory.x']);
            csvwrite(directory_virtual + "u.csv", [timeVector(1:end-1)' this.virtualHistory.u']);
            csvwrite(directory_virtual + "d.csv", [timeVector(1:end-1)' this.virtualHistory.d']);
            
            % store each cost fun seperately
            costNames = fieldnames(this.controller.costFunctionIndexes);
            for idx = 1:length(costNames)
                costName = costNames{idx};
                
                csvwrite(directory + "costs_" + costName + ".csv", [timeVector(1:end-1)' this.history.costs.(costName)']);
                csvwrite(directory_virtual + "costs_" + costName + ".csv", [timeVector(1:end-1)' this.virtualHistory.costs.(costName)']);
            end
            
            % store each eval fun seperately
            evalNames = fieldnames(this.config.evalFuns);
            for idx = 1:length(evalNames)
                evalName = evalNames{idx};
                
                csvwrite(directory + "eval_" + evalName + ".csv", [timeVector(1:end-1)' this.history.evalValues.(evalName)']);
                csvwrite(directory_virtual + "eval_" + evalName + ".csv", [timeVector(1:end-1)' this.virtualHistory.evalValues.(evalName)']);
            end
            
            csvwrite(directory + "time.csv", timeVector');
            
            % if pareto is activated, store pareto results in subdirectory
            if this.doPareto
                paretoDir = directory + filesep + "pareto" + filesep;
                
                if ~exist(paretoDir, 'dir')
                   mkdir(paretoDir)
                end
                
                csvwrite(paretoDir + "utopias.csv", [timeVector(1:end-1)' this.history.pareto.utopias]); 
                csvwrite(paretoDir + "nadirs.csv", [timeVector(1:end-1)' this.history.pareto.nadirs]); 
                csvwrite(paretoDir + "chosenParameters.csv", [timeVector(1:end-1)' this.history.pareto.chosenParameters]);
                
                fronts = this.history.pareto.fronts;
                paretoParameters = this.history.pareto.paretoParameters;
                chosenSolutions = this.history.pareto.chosenSolutions;
                FDS = this.history.pareto.frontDeterminationScheme;
                save(paretoDir + "fronts.mat", "fronts");
                save(paretoDir + "paretoParameters.mat", "paretoParameters");
                save(paretoDir + "chosenSolutions.mat", "chosenSolutions");
                save(paretoDir + "frontDeterminationScheme.mat", "FDS");
            end
        end
        
        function log(this, message, display)
            % log(message)   Will log the given message into the agent's log file
            %               as well as the global simulation log file
            
            % don't log anything in silentMode
            if this.simulation.silentMode
                return
            end
            
            if nargin < 3
                display = false;
            end
            
            if isempty(this.logFileHandle)
                agentDir = this.simulation.resultDirectory + string(this.name) + filesep;
                if ~exist(agentDir, 'dir')
                    mkdir(agentDir);
                end
                
                % clear log file if it already exists
                if exist(agentDir + "log.txt", 'file')
                    tempHandle = fopen(agentDir + "log.txt", "w");
                    fwrite(tempHandle, '');
                    fclose(tempHandle);
                end
                
                this.logFileHandle = fopen(agentDir + "log.txt", "a+");
            end
            
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
            fprintf(this.logFileHandle, "[%s] %s\n", timestamp, message);
            
            this.simulation.log("\t Agent " + this.name + ": " + message, display);
        end
        
        function logConfig(this)
            % logConfig Logs the agent's and controller's configuration in a human readable text file
            
            separator = '-------------------------------';
            
            outputString = string( sprintf('Configuration for Agent %s:\n %s \n\n', this.name, separator) );
            defaultConfig = this.getDefaultConfig();
            
            outputString = outputString + structToString(this.config, defaultConfig);
            outputString = outputString + string( sprintf('\n%s\nConfiguration for %s controller: \n %s \n', separator, this.controller.type, separator) );
            outputString = outputString + structToString(this.controller.config, this.controller.getDefaultConfig());
            
            agentDir = this.simulation.resultDirectory + string(this.name) + filesep;
            fileHandle = fopen(agentDir + "config.txt", "w+");
            fprintf(fileHandle, outputString);
            fclose(fileHandle);
            
            function out = structToString(s, defaults, prefix)
                if nargin < 3
                    prefix = '';
                end
                
                out = "";
                names = fieldnames(s);
                for i=1:length(names)
                    name = names{i};
                    value = s.(name);

                    % check if field has default config
                    if isfield(defaults, name)
                        defaultValue = defaults.(name);
                        out = out + sprintf('%s%s:\n \t %s \n \t (default %s)\n', prefix, name, toString(value), toString(defaultValue));
                        if isstruct(value)
                           out =  out + structToString(value, struct, [name '.']);
                        end
                    else
                        out = out + sprintf('%s%s:\n \t %s \n', prefix, name, toString(value) );
                    end
                end
            end
            
            function s = toString(value)
                if isempty(value)
                    if iscell(value)
                        s = '{}';
                    elseif isstruct(value)
                        s = '(empty struct)';
                    else
                        s = '[]';
                    end
                    return;
                end
                
                if isnumeric(value) && length(value) > 1
                    s = sprintf('[ %s ]', num2str(value) );
                    return;
                end
                
                if islogical(value)
                    if value == 1
                        s = 'true';
                    else
                        s = 'false';
                    end
                    
                    return;
                end
                
                if isnumeric(value)
                    s = num2str(value);
                    return;
                end
                
                if ischar(value)
                    s = "'" + value + "'";
                    return;
                end
                
                if isstring(value)
                    s = '"' + value + '"';
                    return;
                end
                
                if iscell(value)
                    s = ['{' toString(value{1})];
                    for j=2:numel(value)
                        s = [s ', ' toString(value{j})];
                    end
                    s = [s '}'];
                    
                    return;
                end

                if isstruct(value)
                    s = '(structure)';
                    return;
                end
                
                if isa(value, 'function_handle')
                    s = sprintf('%s', func2str(value));
                    return;
                end
                
            end
           
        end
        
        function setTimestepSeeds( this, seeds )
            this.timestepSeeds = seeds;
        end
        
        function updateRng(this)
            rng( this.timestepSeeds( this.status.k + 1 ) );
        end
    end
    
    methods (Static)
        function config = getDefaultConfig()
            config = struct;
            
            % basic configuration
            config.T_s = [];
            config.solver = 'gurobi';
            config.evalFuns = struct;
            config.disturbanceMeasured = false;
            config.debugMode = false;
            config.testFeasibility = false;
            config.pauseOnError = false;
            config.solverOptions = {};
            config.reuseNegotiationResult = false;
            
            config.doParetoOnlyOnce = false;
        end
    end
end