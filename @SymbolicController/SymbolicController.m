classdef SymbolicController < Controller
    properties

        % YALMIP optimizer object
        optimizer
        
    end
    
    properties(Access = protected)
        symPreviousU
        symConsiderPreviousU
    end
    
    methods
        function obj = SymbolicController( numScenarios )
            obj@Controller(numScenarios);
            obj.type = 'symbolic';
            
            % force using optimize() instead of optimizer
            obj.config.forceOptimize = false;
            
            % enable verbosity (debug = 1, verbose = 2) without forcing optimizer
            obj.config.verbose = false;
            
        end

        
        function compile(obj, model, agent, yalmipOptions)
            % compile   Compiles a controller and creates and optimizer object
            %           Builds constraints, collects slacks and
            %           constraints, builds the symbolic cost expression
            %           and initialisies and optimiser object
            % 
            % model         model struct
            % T_s           vector of sample times over horizon
            % yalmipOptions yalmip options object with solver configuration
            
            % if model has implicit predictions, add dynamic constraints
            if model.implicitPrediction
                predConstraints = obj.buildPredictionConstraints(model, model.d, obj.paramSyms, agent.config.T_s);
                obj.constraints = [obj.constraints; predConstraints];
            end
            
            % symbols for the delta constraint du(0) = u(0|k) - u(k-1)
            obj.symPreviousU = sdpvar(model.n_u, 1);
            obj.symConsiderPreviousU = sdpvar;
            
            % build box and delta constraints from list
            obj.buildBoxConstraints(model);
            obj.buildDeltaConstraints(model, agent.config.T_s);
            
            compile@Controller(obj, model, agent, yalmipOptions);
            
            % evaluate functions for implicitly defined constraints, i.e. via function handles
            for i=1:length(obj.implicitConstraints)
                implicitConstraint = obj.implicitConstraints{i};
                
                % if implicit constraint doesn't take scenario s as 4th argument, assume implicit constraint handles
                % scenarios itself
                if nargin( implicitConstraint ) == 3
                    constraint = implicitConstraint(model, obj.paramSyms, obj.slackVariables);
                    
                % otherwise create implicit constraint for each scenario
                else
                    constraint = [];
                    for s=1:obj.numScenarios
                        constraint = [constraint; implicitConstraint(model, obj.paramSyms, obj.slackVariables, s)];
                    end
                end
                
                obj.constraints = [obj.constraints; constraint];
            end
            
            % build cost expression
            expr = 0;
            for i = 1:numel(obj.costFunctions)
                costFunction = obj.costFunctions{i};
                
                obj.costExpressions{i} = costFunction.buildExpression(             ...
                    model.x, model.u, model.d, obj.paramSyms, obj.numScenarios, obj.slackVariables, agent.config.T_s  ...
                );
            
                expr = expr + obj.weightSyms(i) * obj.costExpressions{i};
            end
            
            if isa(expr, 'sdpvar')
                obj.costExpression = expr;
            else
                obj.costExpression = [];
            end
            
            
            % collect all symbolic expressions into one cell array to pass
            % to optimizer
            if ~isempty(obj.costExpression)
                optimizerSymbols = {obj.weightSyms, model.x0};
            else
                optimizerSymbols = {model.x0};
            end
            
            if model.n_d > 0
                for s=1:obj.numScenarios
                    optimizerSymbols{end+1} = model.d{s};
                end
            end
            
            paramNames = fieldnames(obj.paramSyms);
            for idx = 1:length(paramNames)
                name = paramNames{idx};
                scenarioDependent = obj.paramConfig.(name){3};
                for s=1:obj.numScenarios
                    % do not repeat scenario dependent variables
                    % because yalmip does not allow multiple mentions of
                    % the same decision variable in optimizer construction
                    if s > 1 && ~scenarioDependent
                        break;
                    end
                    
                    optimizerSymbols{end+1} = obj.paramSyms.(name){s};
                end
            end
            
            % add symbols for delta constraint on u(0|k)
            optimizerSymbols{end+1} = obj.symPreviousU;
            optimizerSymbols{end+1} = obj.symConsiderPreviousU;
            
            % define what optimizer should output
            output = {model.u};
            
            % optimizer has to deliver all slack variables, since value()
            % does not work with optimizer
            slackVariableNames = fieldnames(obj.slackVariables);
            for idx=1:length(slackVariableNames)
                output{end+1} = obj.slackVariables.(slackVariableNames{idx});
            end
            
            obj.optimizer = optimizer(obj.constraints, obj.costExpression, yalmipOptions, optimizerSymbols, output);
            
        end
        
        function [uPred, slackValues, code] = getInput(obj, x0, agent, additionalConstraints, additionalExpression)
            % [uPred, slackValues, code] = getInput Retrieves an input trajectory and the realised values of the slack variables
            %                               as well as the yalmip problem code
            % 
            %   x0                      assumed initial state
            %   dPred                   scenarios of predictions for disturbances over horizon
            %   paramValues             values for the parameters of the optimization problem            
            %   agent                   calling agent
            %   additionalConstraints   (optional) additional constraints to be appended to all other constraints,
            %                           must not introduce new symbols
            %   additionalExpression   (optional) additional cost expression to be appended to total costExpression
            
            
            if nargin <  5
                additionalConstraints = [];
            end
            
            if nargin < 6
                additionalExpression = [];
            end
            
            weights = obj.defaultWeights;
            
            % If feasibility test is enabled, test before trying to solve
            % actual problem
            if agent.config.testFeasibility
                obj.testFeasibility(x0, agent, additionalConstraints);
            end
            
            %   collect values to replace symbolic variables
            [values, valuesVector] = obj.collectValues(x0, agent);
            
            % if temporary constraints shall be set, optimizer cannot be used
            if ( ~isempty(obj.callbackTempConstraints)      ...
                    || agent.config.debugMode               ...
                    || ~isempty(additionalConstraints)      ...
                    || ~isempty(additionalExpression)       ...
                    || obj.config.forceOptimize             ...
                )
                % add temporary constraints from callback, if available
                if ~isempty(obj.callbackTempConstraints)
                    tempConstraints = obj.callbackTempConstraints(agent);
                    optimizeConstraints = [obj.constraints; tempConstraints];
                else
                    optimizeConstraints = obj.constraints;
                end
                
                % add additional constraints if they're not empty
                if ~isempty(additionalConstraints)
                    optimizeConstraints = [optimizeConstraints; additionalConstraints];
                end
                
                optimizeCost = obj.costExpression;
                
                if ~isempty(additionalExpression)
                    optimizeCost = optimizeCost + additionalExpression;
                end
                
                % collect symbols manually that shall be replaced by values
                symbols = obj.collectSymbols( agent );
                
                optimizeConstraints = replace(optimizeConstraints, symbols, valuesVector);
                optimizeCost = replace(optimizeCost, symbols, valuesVector);

                
                agent.log("solving using optimize()");
                result = optimize(optimizeConstraints, optimizeCost, obj.yalmipOptions);
                uPred = value(agent.model.u);
                
                slackValues = struct;
                slackVariableNames = fieldnames(obj.slackVariables);
                for idx = 1:length(slackVariableNames)
                    name = slackVariableNames{idx};
                    slackValues.(name) = value( obj.slackVariables.(name) );
                end
                code = result.problem;
                
                message = yalmiperror(result.problem);
            else
                agent.log("solving using optimizer");

                solver = obj.optimizer(values, 'nosolve');

                if obj.config.verbose
                    solver.options.debug = 1;
                    solver.options.verbose = 2;
                end
                
                % problem is structurally infeasible
                if ~isa(solver, 'optimizer')
                    agent.log("problem structurally infeasible");
                    
                    code = 1; % infeasible
                    variables = solver;
                else
                    % skip solver call, retrieve variables from cell
                    [variables, code, ~, ~, ~, diagnostics] = solver([]);
                end
                
                if iscell(variables)
                    uPred = variables{1};
                else
                    uPred = variables;
                end
                
                slackValues = struct;
                slackVariableNames = fieldnames(obj.slackVariables);
                for idx = 1:length(slackVariableNames)
                    name = slackVariableNames{idx};
                    slackValues.(name) = variables{idx+1};
                end
                
                message = yalmiperror(code);
            end
            
            agent.log("Solver message: " + message);
               
            % if post optimisation callback is set, update uPred accordingly
            if ~isempty(obj.callbackManipulateInput)
                uPred = obj.callbackManipulateInput(obj, uPred, agent);
            end
        end
        
        function result = testFeasibility(obj, x0, agent, additionalConstraints)
            % testFeasibility   Performs a feasibility test on the the constraints
            % Solves an optimization problem with empty objective function
            % but all the constraints. If this problem can be solved, it
            % means that the problem becomes infeasible due to some problem
            % in the objective function
            if nargin <  5
                additionalConstraints = [];
            end
            
            [~, valuesVector] = obj.collectValues(x0, agent);
            symbols = obj.collectSymbols( agent );
            
            
            % add temporary constraints from callback, if available
            if ~isempty(obj.callbackTempConstraints)
                tempConstraints = obj.callbackTempConstraints(agent);
                optimizeConstraints = [obj.constraints; tempConstraints];
            else
                optimizeConstraints = obj.constraints;
            end
            
            % add additional constraints if they're not empty
            if ~isempty(additionalConstraints)
                optimizeConstraints = [optimizeConstraints; additionalConstraints];
            end
            
            optimizeConstraints = replace(optimizeConstraints, symbols, valuesVector);
            
            % perform feasibility test
            result = optimize(optimizeConstraints, [], obj.yalmipOptions);
            check(optimizeConstraints);
            agent.log("Feasibility test: " + result.info, true);
        end
    end
    
    methods (Access = protected)
        function [values, valuesVector] = collectValues(obj, x0, agent)
            [values, valuesVector] = collectValues@Controller(obj, x0, agent);
            
            if( size(agent.history.u, 2) > 0)
                uPrev = agent.history.u(:, end);
                considerPreviousU = 1;
            else
                uPrev = zeros(agent.model.n_u, 1);
                considerPreviousU = 0;
            end
            
            values{end+1} = uPrev;
            values{end+1} = considerPreviousU;
            
            valuesVector = [valuesVector; uPrev(:); considerPreviousU];
        end
        
        function [symbols] = collectSymbols(obj, agent)
            symbols = collectSymbols@Controller(obj, agent);
            symbols = [symbols; obj.symPreviousU(:), obj.considerPreviousU];
        end
        
        function buildBoxConstraints(obj, model)
            % buildBoxConstraints  Adds box constraints using addConstraint
            %                      during compile
            
            % build constraints from prenoted box constraints
            for i=1:numel(obj.boxConstraintsTemp)
                [variable, index, lb, ub] = obj.boxConstraintsTemp{i}{:};
                
                % make sure we are dealing with a char
                variable = char(variable);
                
                tag = [ 'box on ' variable num2str(index(1)) '..' num2str(index(end)) ];
                
                if variable == 'x'
                    variableSym = model.(variable){1};
                else
                    variableSym = model.(variable);
                end
                    
                % get number 
                if variable == 'x'
                    N_horz = size(variableSym(:, 2:end), 2);
                    horz_idx = 2:N_horz+1;
                else
                    N_horz = size(variableSym, 2);
                    horz_idx = 1:N_horz;
                end
                
                % lb is symbol, then get symbolic expression
                if isstring(lb) || ischar(lb)
                    lb = obj.paramSyms.(lb);
                end
                % if lb is symbol, create temporary variable lb_ to access
                % size information
                if iscell(lb)
                    lb_ = lb{1};
                else
                    lb_ = lb;
                end
                
                % ub is symbol, then get symbolic expression
                if isstring(ub) || ischar(ub)
                    ub = obj.paramSyms.(ub);
                end
                % if ub is symbol, create temporary variable ub_ to access
                % size information
                if iscell(ub)
                    ub_ = ub{1};
                else
                    ub_ = ub;
                end
                
                if ~all( size(lb_) == [length(index), 1]) && ~all( size(lb_) == [length(index), N_horz])
                    warning("PARODIS Controller:buildBoxConstraints dimensions of LB do not fit");
                end
                
                if ~all( size(ub_) == [length(index), 1]) && ~all( size(ub_) == [length(index), N_horz])
                    warning("PARODIS Controller:buildBoxConstraints dimensions of UB do not fit");
                end
                
                if iscell( model.(variable) )
                    N_S = length( model.(variable) );
                else
                    N_S = 1;
                end
                
                for s=1:N_S
                    % if lb/ub is a constant and no parameter, use same lb/ub for every s
                    if iscell(lb)
                        lb_ = lb{s};
                    else
                        lb_ = lb;
                    end
                    
                    if iscell(ub)
                        ub_ = ub{s};
                    else
                        ub_ = ub;
                    end
                    
                    % roll out LB and scale according to T_s and T_s_ref
                    if size(lb_, 2) == 1
                        lb_ = repmat(lb_, 1, N_horz);
                    end
                    
                    % roll out UB and scale according to T_s and T_s_ref
                    if size(ub_, 2) == 1
                        ub_ = repmat(ub_, 1, N_horz);
                    end
                    
                    if iscell( model.(variable) )
                        variableSym = model.(variable){s};
                    else
                        variableSym = model.(variable);
                    end
                    
                    obj.addConstraint( ( lb_ <= variableSym(index, horz_idx) <= ub_ ):sprintf('%s s = %i', tag, s) );
                end
            end
        end
        
        function constraints = getStateDeltaConstraints(obj, model, index, lb, ub, T_s_ref, T_s)
            tag = [ 'delta on x' num2str(index(1)) '..' num2str(index(end)) ];
            variableSym = model.x{1};
            
            % get horizontal length of variable
            N_horz = size(variableSym, 2);

            % lb is symbol, then get symbolic expression
            if isstring(lb) || ischar(lb)
                lb = obj.paramSyms.(lb);
            end
            
            % if lb is symbol, create temporary variable lb_ to access
            % size information
            if iscell(lb)
                lb_ = lb{1};
            else
                lb_ = lb;
            end

            % ub is symbol, then get symbolic expression
            if isstring(ub) || ischar(ub)
                ub = obj.paramSyms.(ub);
            end
            % if ub is symbol, create temporary variable ub_ to access
            % size information
            if iscell(ub)
                ub_ = ub{1};
            else
                ub_ = ub;
            end

            if ~all( size(lb_) == [length(index), 1]) && ~all( size(lb_) == [length(index), N_horz - 1])
                warning("PARODIS Controller:getStateDeltaConstraints dimensions of LB do not fit");
            end

            if ~all( size(ub_) == [length(index), 1]) && ~all( size(ub_) == [length(index), N_horz - 1])
                warning("PARODIS Controller:getStateDeltaConstraints dimensions of UB do not fit");
            end

            % vector for scaling dx to appropriate time steps
            scale = T_s(1:N_horz-1)/T_s_ref; % dx/du consider N_horz-1 many steps, and N_horz is shorter for du

            N_S = length( model.x );

            constraints = [];
            for s=1:N_S
                % if lb/ub is a constant and no parameter, use same lb/ub for every s
                if iscell(lb)
                    lb_ = lb{s};
                else
                    lb_ = lb;
                end

                if iscell(ub)
                    ub_ = ub{s};
                else
                    ub_ = ub;
                end

                % roll out LB and scale according to T_s and T_s_ref
                if size(lb_, 2) == 1
                    lb_ = repmat(lb_, 1, N_horz - 1);
                end
                lb_ = lb_ .* scale;

                % roll out UB and scale according to T_s and T_s_ref
                if size(ub_, 2) == 1
                    ub_ = repmat(ub_, 1, N_horz - 1);
                end
                ub_ = ub_ .* scale;

                variableSym = model.x{s};

                % yalmip constraint expression for x(n+1|k) - x(n|k)
                constraint = lb_ <= variableSym(index, 2:end) - variableSym(index, 1:end-1) <= ub_;
                
                constraints = [constraints; constraint:sprintf('%s s = %i', tag, s)];
            end
        end
        
        function constraints = getInputDeltaConstraints(obj, model, index, lb, ub, T_s_ref, T_s)
            tag = [ 'delta on u' num2str(index(1)) '..' num2str(index(end)) ];
            variableSym = model.u;
            
            % get horizontal length of variable
            N_horz = size(variableSym, 2);

            % lb is symbol, then get symbolic expression
            if isstring(lb) || ischar(lb)
                lb = obj.paramSyms.(lb);
            end
            
            % if lb is symbol, create temporary variable lb_ to access
            % size information
            if iscell(lb)
                lb_ = lb{1};
            else
                lb_ = lb;
            end

            % ub is symbol, then get symbolic expression
            if isstring(ub) || ischar(ub)
                ub = obj.paramSyms.(ub);
            end
            % if ub is symbol, create temporary variable ub_ to access
            % size information
            if iscell(ub)
                ub_ = ub{1};
            else
                ub_ = ub;
            end

            if ~all( size(lb_) == [length(index), 1]) && ~all( size(lb_) == [length(index), N_horz])
                warning("PARODIS Controller:getInputDeltaConstraint dimensions of LB do not fit");
            end

            if ~all( size(ub_) == [length(index), 1]) && ~all( size(ub_) == [length(index), N_horz])
                warning("PARODIS Controller:getInputDeltaConstraint dimensions of UB do not fit");
            end

            % vector for scaling dx to appropriate time steps
            scale = [T_s(1) T_s(1:N_horz-1)]/T_s_ref; % for scaling du(0) = u(0|k) - u(k-1), we also need to scale with T_s(1), so we double it
            N_S = length( model.x );

            constraints = [];
            for s=1:N_S
                % if lb/ub is a constant and no parameter, use same lb/ub for every s
                if iscell(lb)
                    lb_ = lb{s};
                else
                    lb_ = lb;
                end

                if iscell(ub)
                    ub_ = ub{s};
                else
                    ub_ = ub;
                end

                % roll out LB and scale according to T_s and T_s_ref
                if size(lb_, 2) == 1
                    lb_ = repmat(lb_, 1, N_horz);
                end
                lb_ = lb_ .* scale;

                % roll out UB and scale according to T_s and T_s_ref
                if size(ub_, 2) == 1
                    ub_ = repmat(ub_, 1, N_horz);
                end
                ub_ = ub_ .* scale;

                % yalmip constraint expression for u(n+1|k) - u(n|k) for n=1 .. N_pred-2
                constraint = lb_(:, 2:end) <= variableSym(index, 2:end) - variableSym(index, 1:end-1) <= ub_(:, 2:end);
                constraints = [constraints; constraint:sprintf('%s s = %i', tag, s)];
                
                % we need to consider du(0) = u(0|k) - u(k-1) separately, because if no previous u exists, i.e. in k=0
                % we must deactivate the corresponding constraint
                constraint_on_prev = lb_(:, 1) <= obj.symConsiderPreviousU * (variableSym(index, 1) - obj.symPreviousU(index, 1)) <= ub_(:, 1);
                constraints = [constraints; constraint_on_prev:sprintf('%s on u(0|k) s = %i', tag, s)];
            end
        end
        
        function buildDeltaConstraints(obj, model, T_s)
            % buildBoxConstraints  Adds delta constraints using addConstraint
            %                      during compile
            
            % build constraints from prenoted delta constraints
            for i=1:numel(obj.deltaConstraintsTemp)
                [variable, index, lb, ub, T_s_ref] = obj.deltaConstraintsTemp{i}{:};
                
                % extract second letter from dx or du
                variable = variable{1}(2);
                
                if strcmp(variable, 'u')
                    constraints = obj.getInputDeltaConstraints(model, index, lb, ub, T_s_ref, T_s);
                else
                    constraints = obj.getStateDeltaConstraints(model, index, lb, ub, T_s_ref, T_s);
                end
                
                obj.addConstraint( constraints );
            end
        end
    end
end