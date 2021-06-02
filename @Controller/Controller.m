classdef Controller < handle
    properties
        % symbolic variables for cost function weights
        weightSyms
        
        % symbolic variables for parameters
        paramSyms = struct;
       
        % source configuration for parameters
        paramConfig = struct;
        
        % fixed YALMIP constraints
        constraints = [];
        
        % YALMIP constraints defined through function handles
        implicitConstraints = {};
        
        % cell array of box constraint descriptions
        boxConstraintsTemp = {};
        
        % cell array of delta constraint descriptions
        deltaConstraintsTemp = {};
        
        % callback to set temporary constraints right before getInput call
        callbackTempConstraints
        
        % callback to manipulate input retrieved from getInput
        callbackManipulateInput

        
        % YALMIP options object
        yalmipOptions
        
        % source for predicted disturbances
        predDisturbanceSource = []
        
        % source for realised disturbances
        realDisturbanceSource = NaN
        
        % default weights assumed for cost functions
        defaultWeights = [];
        
        % number of scenarios to be considered
        numScenarios
        
        % list of cost function instances
        costFunctions = {};
        
        % mapping for cost function name to the internal index
        costFunctionIndexes = struct;
        
        % YALMIP sdpvar expression of total optimization cost function
        costExpression
        
        % YALMIP expression for each cost function
        costExpressions
        
        % slack variables introduced by cost functions
        slackVariables = struct;
        
        type
        
        config = struct;
    end
    
    properties (Access = protected)
        isCompiled = false;
    end
    
    methods
        function obj = Controller( numScenarios )
            obj.numScenarios = numScenarios;
        end
        
        function addCostFunction(obj, name, costFunctionInstance, defaultWeight )
            if nargin < 4
                defaultWeight = 1;
            end
            
            obj.costFunctions{end+1} = costFunctionInstance;
            obj.defaultWeights(end+1) = defaultWeight;
            obj.costFunctionIndexes.(name) = length(obj.costFunctions);
        end
        
        % creates sdp variables for parameters and store config for parameter
        function paramSymbol = addParam(obj, name, dimensions, source, scenarioDependent)
            obj.paramSyms.(name) = cell(obj.numScenarios, 1);
            obj.paramConfig.(name) = {dimensions, source, scenarioDependent};
            
            % define symbolic variables for each scenario
            for s=1:obj.numScenarios
                if s > 1 && ~scenarioDependent
                    obj.paramSyms.(name){s} = obj.paramSyms.(name){1};
                    
                    continue;
                end
                
                obj.paramSyms.(name){s} = sdpvar(dimensions(1), dimensions(2));
            end
            
            paramSymbol = obj.paramSyms.(name);
        end
        
        function addBoxConstraint(obj, variable, index, lb, ub)
            if variable ~= 'x' && variable ~= 'u'
                w
                return
            end
            
            obj.boxConstraintsTemp{end+1} = {variable, index, lb, ub};
        end
        
        function addDeltaConstraint(obj, variable, index, lb, ub, Ts)
            if variable ~= "dx" && variable ~= "du"
                warning("PARODIS Controller:addDeltaConstraint variable not supported (dx or du only)");
                return
            end
            
            obj.deltaConstraintsTemp{end+1} = {string(variable), index, lb, ub, Ts};
        end
        
        function addConstraint(obj, arg)
            % if constraint is given implicitly as function handle, store until compilation
            if isa(arg, 'function_handle')
                obj.implicitConstraints = [obj.implicitConstraints; {arg}];
            % otherwise add immediately
            elseif isa(arg, 'constraint') || isa(arg, 'lmi')
                 obj.constraints = [obj.constraints; arg];
            end
        end
        
        function slack = addSharedSlack(obj, name, dimensions, full)
            if nargin < 4
                slack = sdpvar(dimensions(1), dimensions(2));
            elseif full
                slack = sdpvar(dimensions(1), dimensions(2), 'full');
            end
            
            obj.slackVariables.(name) = slack;
        end
        
        function dPred = getPredDisturbance(obj, T, callingAgent, agents)
            dPred = callingAgent.simulation.sourceManager.getDataFromSource(T, obj.predDisturbanceSource, callingAgent, agents, obj.numScenarios);
        end
        
        function dReal = getRealDisturbance(obj, T, callingAgent, agents)
            % retrieve real disturbance from real disturbance source
            % by requiring only one scenario and unwrapping result from cell
            dReal = callingAgent.simulation.sourceManager.getDataFromSource(T, obj.realDisturbanceSource, callingAgent, agents, 1);
            dReal = dReal{1};
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
            
            % add slacks and constraints from cost functions
            for costFunctionCell = obj.costFunctions
                costFunction = costFunctionCell{1};
                
                slacks = costFunction.getSlacks(model, agent, obj.paramSyms);
                if ~isa(slacks, 'struct')
                    warning("PARODIS Controller:compile cost function getSlacks must return struct");
                    continue;
                end
                obj.slackVariables = mergeStructs(obj.slackVariables, slacks);
            end
            
            % if controller is symbolic, collect constraints symbolically
            % done in seperate loop so all cost functions can access all
            % slack variables
            if isequal(obj.type, 'symbolic')
                for costFunctionCell = obj.costFunctions
                    costFunction = costFunctionCell{1};
                    extraConstraints = costFunction.getConstraints(model, agent, obj.slackVariables, obj.paramSyms);
                    obj.constraints = [obj.constraints; extraConstraints];
                end
            end
            
            % create sdpvar for symbolic weights
            obj.weightSyms = sdpvar(1, length( obj.costFunctions) );
            
            obj.yalmipOptions = yalmipOptions;
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
            
            
            
        end
    end
    
    methods (Static)
        function config = getDefaultConfig()
            config = struct;
        end
    end
    
    methods (Access = protected)
        function constraints = buildPredictionConstraints(obj, model, d, parameters, T_s)
            % buildPredictionConstraints    Adds implicit prediction
            %   constraints during compile, i.e. equality constraints
            %   x(n+1|k) == f_n(x(n|k), u(n|k), d(n|k))
            
            N_pred = length(T_s);
            odes = model.odes;
            
            constraints = [];
            
            for s=1:obj.numScenarios
                params = extractScenario(parameters, s);
                
                for k=1:N_pred
                    tag = char( sprintf("x(%i) = f_%i(x(%i), u(%i), d(%i))", k, s, k-1, k-1, k-1) );
                    if model.parameterVariant
                        dynConstraint = (model.x{s}(:, k+1) == odes{k}(model.x{s}(:, k), model.u(:, k), d{s}(:, k), k, params)):tag;
                    else
                        dynConstraint = (model.x{s}(:, k+1) == odes{k}(model.x{s}(:, k), model.u(:, k), d{s}(:, k))):tag;
                    end
                    
                    constraints = [constraints; dynConstraint];
                end
            end
        end
        
        function [values, valuesVector] = collectValues(obj, x0, agent)
            if ~isempty(obj.costFunctions)
                values = {obj.defaultWeights, x0};
                valuesVector = [obj.defaultWeights(:); x0(:)];
            else
                values = {x0};
                valuesVector = [x0(:)];
            end
            
            if agent.model.n_d > 0
                for s=1:obj.numScenarios
                    values{end+1} = agent.status.dPred{s};
                    valuesVector = [valuesVector; agent.status.dPred{s}(:)];
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
                    
                    values{end+1} = agent.status.paramValues.(name){s};
                    valuesVector = [valuesVector; agent.status.paramValues.(name){s}(:)];
                end
            end
        end
        
        function [symbols] = collectSymbols(obj, agent)
            symbols = [obj.weightSyms(:); agent.model.x0(:)];
            
            if agent.model.n_d > 0
                for s=1:obj.numScenarios
                    symbols = [symbols; agent.model.d{s}(:)];
                end
            end

            paramNames = fieldnames(obj.paramSyms);
            for idx = 1:length(paramNames)
                name = paramNames{idx};
                scenarioDependent = obj.paramConfig.(name){3};
                for s=1:obj.numScenarios
                    if s > 1 && ~scenarioDependent
                        break;
                    end

                    symbols = [symbols; obj.paramSyms.(name){s}(:)];
                end
            end
        end
    end
end