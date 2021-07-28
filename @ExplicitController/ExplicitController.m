classdef ExplicitController < Controller
    properties
    end
    
    methods
        function obj = ExplicitController( numScenarios )
            if nargin < 1
                numScenarios = 1;
            end
            
            obj@Controller(numScenarios);
            obj.type = 'explicit';
        end
        
        function addConstraint(obj, arg)
            if isa(arg, 'constraint')
                warning("PARODIS ExplicitController:addConstraint You should avoid using explicit parametric constraint expressions when using the ExplicitController." + newline ...
                + "Using explicit constraint expressions requires replacing parameters using YALMIP's replace() function which is very, very slow. " + newline + ...
                "Consider wrapping the constraint in a (anonymous) function" );
            end
            
            addConstraint@Controller(obj, arg);
        end
        
        function [uPred, slackValues, code] = getInput(obj, x0, uPrev, agent, additionalConstraints, additionalExpression)
            % [uPred, slackValues, code] = getInput Retrieves an input trajectory and the realised values of the slack variables
            %                               as well as the yalmip problem code
            % 
            %   x0                      assumed initial state
            %   uPrev                   previously applied input u, i.e. u(k-1)
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
            
            if agent.config.warmstart && ~isempty(agent.previousStatus)
                if ~isempty(agent.previousStatus.xPred{1})
                    assign(agent.model.x{1}, [agent.previousStatus.xPred{1}(:, 2:end),...
                        agent.previousStatus.xPred{1}(:, end)]);
                end
                if ~isempty(agent.previousStatus.uPred)
                    assign(agent.model.u, [agent.previousStatus.uPred(:, 2:end),...
                        agent.previousStatus.uPred(:, end)]);
                end
            end
            
            [optimizeConstraints, costExpressions] = obj.prepareProblem(x0, uPrev, agent, additionalConstraints);
            
            expr = 0;
            for i = 1:numel(obj.costFunctions)
                % don't add expression if weight is 0 to save time
                % and avoid numerical problems
                if obj.defaultWeights(i) == 0
                    continue;
                end
                
                expr = expr + obj.defaultWeights(i) * costExpressions{i};
            end
            
            if isa(expr, 'sdpvar')
                obj.costExpression = expr;
            else
                obj.costExpression = [];
            end
            
            optimizeCost = obj.costExpression;
            
            if ~isempty(additionalExpression)
                optimizeCost = optimizeCost + additionalExpression;
            end
            
            % If feasibility test is enabled, test before trying to solve
            % actual problem
            if agent.config.testFeasibility
                obj.testFeasibility(agent, optimizeConstraints);
            end
           
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

            
            agent.log("Solver message: " + message);
            
            % if post optimisation callback is set, update uPred accordingly
            if ~isempty(obj.callbackManipulateInput)
                uPred = obj.callbackManipulateInput(obj, uPred, agent);
            end
        end
        
        function [optimizeConstraints, costExpressions] = prepareProblem(obj, x0, uPrev, agent, additionalConstraints)
            paramValues = agent.status.paramValues;
            
            model = obj.prepareModel(agent, x0);
            
            constraints = [];
            
            % explicitly defined constraints, i.e. fully parametrised ones have to be replaced with constraints
            % containing the realised parameter values at the given time step
            % this is very slow and should be avoided by only using implicitly defined constraints, i.e. by wrapping
            % them in (anonymous) functions and adding those via addConstraint
            if ~isempty(obj.constraints)
                % replace params in all fixed constraints
                [~, valuesVector] = obj.collectValues(x0, agent);
                symbols = obj.collectSymbols( agent );
                
                constraints = replace(obj.constraints, symbols, valuesVector);
            end
            
            % evaluate functions for implicitly defined constraints, i.e. via function handles
            for i=1:length(obj.implicitConstraints)
                implicitConstraint = obj.implicitConstraints{i};
                
                % if implicit constraint doesn't take scenario s as 4th argument, assume implicit constraint handles
                % scenarios itself
                if nargin( implicitConstraint ) == 3
                    constraint = implicitConstraint(model, paramValues, obj.slackVariables);
                    
                % otherwise create implicit constraint for each scenario
                else
                    constraint = [];
                    for s=1:obj.numScenarios
                        constraint = [constraint; implicitConstraint(model, paramValues, obj.slackVariables, s)];
                    end
                end
                
                constraints = [constraints; constraint];
            end
            
            % if model has implicit predictions, state trajectory remains symbolic
            % and bound by equality constraints
            if agent.model.implicitPrediction
                predictionConstraints = obj.buildPredictionConstraints(model, agent.status.dPred, agent.status.paramValues, agent.config.T_s);
                constraints = [constraints; predictionConstraints];
            end
            
            % build constraints with actual values instead of parameters
            boxConstraints = obj.buildBoxConstraints(paramValues, model);
            deltaConstraints = obj.buildDeltaConstraints(paramValues, uPrev, model, agent.config.T_s);
            slackConstraints = [];

            
            for costFunctionCell = obj.costFunctions
                costFunction = costFunctionCell{1};
                
                extraConstraints = costFunction.getConstraints(model, agent, obj.slackVariables, paramValues);
                slackConstraints = [slackConstraints; extraConstraints];
            end

            optimizeConstraints = [constraints; boxConstraints; deltaConstraints; slackConstraints];
            
            costExpressions = {};

            for i = 1:numel(obj.costFunctions)
                costFunction = obj.costFunctions{i};
                
                % don't build expression if weight is 0 to save time
                % and avoid numerical problems
                if obj.defaultWeights(i) == 0
                    costExpressions{i} = 0;
                    continue;
                end
                
                costExpressions{i} = obj.prepareExpression(costFunction, model, agent);
            end

            % add temporary constraints from callback, if available
            if ~isempty(obj.callbackTempConstraints)
                tempConstraints = obj.callbackTempConstraints(agent);
                optimizeConstraints = [optimizeConstraints; tempConstraints];
            end
                
            % add additional constraints if they're not empty
            if ~isempty(additionalConstraints)
                optimizeConstraints = [optimizeConstraints; additionalConstraints];
            end
        end
        
        function expr = prepareExpression(obj, costFunction, model, agent)
            dPred = agent.status.dPred;
            paramValues = agent.status.paramValues;
            
            expr = costFunction.buildExpression(                      ...
                model.x, model.u, dPred, paramValues, obj.numScenarios, obj.slackVariables, agent.config.T_s    ...
            );
        end
        
        function model = prepareModel(obj, agent, x0)
            dPred = agent.status.dPred;
            
            model = agent.model;
            
            % if implicit prediction form, only replace x0 with actual
            % value, leave rest of trajectory as decision variable
            if agent.model.implicitPrediction
                for s=1:length(model.x)
                    model.x{s}(:, 1) = x0;
                end
            else
                % otherwise, get explicit expression for x{s} in terms of
                % available values
                x_explicit = agent.predictTrajectory(agent.model.u, dPred, x0);
                model.x = x_explicit;
            end
        end
        
        function result = testFeasibility(obj, agent, optimizeConstraints)
            % testFeasibility   Performs a feasibility test on the the constraints
            % Solves an optimization problem with empty objective function
            % but all the constraints. If this problem can be solved, it
            % means that the problem becomes infeasible due to some problem
            % in the objective function
            
            % perform feasibility test
            result = optimize(optimizeConstraints, [], obj.yalmipOptions);
            check(optimizeConstraints);
            agent.log("Feasibility test: " + result.info, true);
        end
    end
    
    methods (Access = protected)
        
        function constraints = buildBoxConstraints(obj, paramValues, model)
            % buildBoxConstraints  Adds box constraints using addConstraint
            %                      during compile
            
            constraints = [];
            
            % build constraints from prenoted box constraints
            for i=1:numel(obj.boxConstraintsTemp)
                [variable, index, lb, ub] = obj.boxConstraintsTemp{i}{:};
                
                % make sure we are dealing with a char
                variable = char(variable);
                
                tag = [ 'box on ' variable num2str(index(1)) '..' num2str(index(end)) ];
                

                variableSym = model.(variable);
                    
                % get number 
                if variable == 'x'
                    N_horz = size(variableSym{1}(:, 2:end), 2);
                    horz_idx = 2:N_horz+1;
                else
                    N_horz = size(variableSym, 2);
                    horz_idx = 1:N_horz;
                end
                
                % lb is symbol, then get symbolic expression
                if isstring(lb) || ischar(lb)
                    lb = paramValues.(lb);
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
                    ub = paramValues.(ub);
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
                
                if iscell( variableSym )
                    N_S = length( variableSym );
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
                        symbol = variableSym{s};
                    else
                        symbol = variableSym;
                    end
                    
                    constraints = [constraints; ( lb_ <= symbol(index, horz_idx) <= ub_ ):sprintf('%s s = %i', tag, s) ];
                end
            end
        end
        
        function constraints = getStateDeltaConstraints(obj, model, paramValues, index, lb, ub, T_s_ref, T_s)
            tag = [ 'delta on x' num2str(index(1)) '..' num2str(index(end)) ];
            variableSym = model.x{1};
            
            % get horizontal length of variable
            N_horz = size(variableSym, 2);

            % lb is symbol, then get symbolic expression
            if isstring(lb) || ischar(lb)
                lb = paramValues.(lb);
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
                ub = paramValues.(ub);
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
        
        function constraints = getInputDeltaConstraints(obj, model, paramValues, uPrev, index, lb, ub, T_s_ref, T_s)
            tag = [ 'delta on u' num2str(index(1)) '..' num2str(index(end)) ];
            variableSym = model.u;
            
            % get horizontal length of variable
            N_horz = size(variableSym, 2);

            % lb is symbol, then get symbolic expression
            if isstring(lb) || ischar(lb)
                lb = paramValues.(lb);
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
                ub = paramValues.(ub);
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
                
                % we need to consider du(0) = u(0|k) - u(k-1) separately, as it requires a parameter for the previous u
                constraint_on_prev = lb_(:, 1) <= (variableSym(index, 1) - uPrev(index)) <= ub_(:, 1);
                constraints = [constraints; constraint_on_prev:sprintf('%s on u(0|k) s = %i', tag, s)];
            end
        end
        
        function constraints = buildDeltaConstraints(obj, paramValues, uPrev, model, T_s)
            % buildBoxConstraints  Adds delta constraints using addConstraint
            %                      during compile
            
            constraints = [];
            % build constraints from prenoted delta constraints
            for i=1:numel(obj.deltaConstraintsTemp)
                [variable, index, lb, ub, T_s_ref] = obj.deltaConstraintsTemp{i}{:};
                
                % extract second letter from dx or du
                variable = variable{1}(2);
                
                if strcmp(variable, 'u')
                    deltaConstraints = obj.getInputDeltaConstraints(model, paramValues, uPrev, index, lb, ub, T_s_ref, T_s);
                else
                    deltaConstraints = obj.getStateDeltaConstraints(model, paramValues, index, lb, ub, T_s_ref, T_s);
                end
                
                constraints = [constraints; deltaConstraints];
            end
        end
        
    end
end