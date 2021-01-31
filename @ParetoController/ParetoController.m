classdef ParetoController < ExplicitController
    %PARETOCONTROLLER Controller for Pareto-optimal MPC
    %   Detailed explanation goes here
    
    properties
        optimizer
        status
        
        interactivityIdx
        evaluatedPoints
        paretoCurrentStep
        paretoMaxStep
    end
    
    methods
        function obj = ParetoController( numScenarios )
            obj@ExplicitController(numScenarios);
            obj.type = 'pareto';
            obj.paretoCurrentStep = [];
            obj.paretoMaxStep = [];
            
            obj.config = ParetoController.getDefaultConfig();
            obj.config.filter = [];
            obj.config.fixedNormValues = [];
            
            obj.status = struct;
            obj.status.utopia = [];
            obj.status.nadir = [];
            obj.status.conflictingObj = [];
            obj.status.independantObj = [];
            obj.status.redundantObj = [];
            obj.status.front = [];
        end
        
        function addCostFunction(obj, name, costFunctionInstance, defaultWeight )
            if nargin < 4
                defaultWeight = 1;
            end
            
            addCostFunction@ExplicitController(obj, name, costFunctionInstance, defaultWeight);
            
            i = length(obj.costFunctions);
            
            obj.config.conflictingObj = 1:i;
            
            % allow pre-setting filters by users via config
            if length(obj.config.filter) < i
                obj.config.filter(i) = NaN;
            end
            
            % allow pre-setting fixed norm values by users via config
            if length(obj.config.fixedNormValues) < i
                obj.config.fixedNormValues(i) = 1;
            end
        end
        
        function [objectiveValues, u, slackValues] = calculateUnnormedObjectiveValues(paretoObj, optimizerOutput, agent)
            % Calculates the unnormed objective values from the output of an optimizer object. This
            % output constists of the input u und the slack variables.
            
            n = numel(paretoObj.costFunctions)-numel(paretoObj.config.ignoreInPareto);
            
            u = optimizerOutput{1};
            
            slackVariableNames = fieldnames(paretoObj.slackVariables);
            slack = optimizerOutput(2:length(slackVariableNames)+1);
            
            slackValues = struct;
            slackVariableNames = fieldnames(paretoObj.slackVariables);
            for idx = 1:length(slackVariableNames)
                name = slackVariableNames{idx};
                slackValues.(name) = slack{idx};
            end
            
            x0 = agent.history.x(:,end);
            d = agent.status.dPred;
            x = agent.predictTrajectory(u, d, x0);
            params = agent.status.paramValues;
            Ns = agent.controller.numScenarios;
            objectiveValues = zeros(1,n);
            
            for objInEP = 1:n
                objectiveValues(1,objInEP) = paretoObj.costFunctions{objInEP}.buildExpression(x,u,d,params,Ns,slackValues,agent.config.T_s);
            end
        end
               
        function [uPred, slackValues, code, paretoStatus] = getInput(this, x0, agent, predefinedParetoParameters, additionalConstraints, additionalExpression)
            % [uPred, slackValues, code, parameters] = getInput Retrieves an input trajectory and the realised values of the slack variables
            %                               as well as the yalmip problem code
            %
            %   x0                      assumed initial state
            %   dPred                   scenarios of predictions for disturbances over horizon
            %   paramValues             values for the parameters of the optimization problem
            %   agent                   calling agent
            %   additionalConstraints   (optional) additional constraints to be appended to all other constraints,
            %                           must not introduce new symbols
            %   additionalExpression   (optional) additional cost expression to be appended to total costExpression
            
            code = 0;
            
            if nargin < 4
                predefinedParetoParameters = [];
            end
            
            if nargin < 5
                additionalConstraints = [];
            end
            
            if nargin < 6
                additionalExpression = [];
            end
            
            this.status.conflictingObj = this.config.conflictingObj;
            
            [optimizeConstraints, costExpressions] = this.prepareProblem(x0, agent, additionalConstraints);
            
            [extremePoints, inputsEP, slacksEP, parametersEP] = this.determineExtremePoints(optimizeConstraints, costExpressions, agent);
            optimizer = this.prepareOptimizer( optimizeConstraints, costExpressions, agent, extremePoints );
            
            % if pareto evaluation shall be done only once per time step (i.e. negotiation)
            % check if parameters have been set already, as they are
            % cleared after completing time step
            if agent.config.doParetoOnlyOnce && ~isempty(agent.status.chosenParameters)
                % TODO: recover/re-evaluate u and slacks from agent.status.chosenParameters
                [recoveredInputs, recoveredSlacks, ~, recoveredParameters] = this.determineFront(                       ...
                    optimizer, agent, this.config.frontDeterminationScheme, extremePoints, predefinedParetoParameters  ...
                    );
                
                
                uPred = recoveredInputs{1};
                slackValues = recoveredSlacks{1};
                chosenParameters = recoveredParameters{1};
            else
                % clear fronts/parameters determined in previous timestep
                this.clear();
                
                [inputs, slacks, front, paretoParameters] = this.determineFront(                    ...
                    agent, optimizer, extremePoints, inputsEP, slacksEP, parametersEP,              ...                
                    predefinedParetoParameters                                                      ...
                );
                
                if isempty(predefinedParetoParameters)
                    if this.config.interactivity % call the interactivity tool and wait for selection
                        this.status.inputs = inputs;
                        agent.simulation.interactivity.currentAgent = agent;
                        agent.simulation.interactivity.app.showScreen;
                        waitfor(agent.simulation.interactivity.app,'waitingFlag');
                        idx = this.interactivityIdx;
                        uPred = inputs{idx,1};
                        slackValues = slacks(idx,:);
                        chosenParameters = paretoParameters(idx,:);
                        chosenSolution = front(idx,:);
                        agent.status.chosenWeights = paretoParameters(idx,:);
                    else
                        [uPred, slackValues, chosenParameters, chosenSolution] = this.chooseSolution( agent, inputs, slacks, front, paretoParameters );
                    end
                else
                    uPred = inputs{end};
                    slackValues = slacks{end};
                    chosenParameters = predefinedParetoParameters;
                    chosenSolution = front(end,:);
                end
            end
            
            paretoStatus = struct;
            paretoStatus.front = front;
            paretoStatus.chosenSolution = chosenSolution;
            paretoStatus.paretoParameters = paretoParameters;
            paretoStatus.chosenParameters = chosenParameters;
            paretoStatus.nadir = this.status.nadir;
            paretoStatus.utopia = this.status.utopia;
        end
        
        function optimizer = prepareOptimizer(this, optimizeConstraints, costExpressions, agent, extremePoints, method)
            % Select the optimizer function to based on the ParetoController config. If the stored
            % string is 'auto' the optimizer corresponding to the front determination scheme is 
            % called. 
            
            if nargin == 5 && isequal(this.config.preperationMethod,'auto')
                method = this.config.frontDeterminationScheme;
            elseif nargin == 5
                method = this.config.preperationMethod;
            end
            
            if isa(method, 'function_handle')
                optimizer = this.config.preparationMethod(this, optimizeConstraints, costExpressions, agent, extremePoints);
            else
                optimizerMethod = "prepare"+method;
                if ismethod(this, optimizerMethod)
                    optimizer = ParetoController.(optimizerMethod)(this, optimizeConstraints, costExpressions, agent, extremePoints);
                else
                    error("No function handle given for front determination scheme! Try 'AWDS', 'NBI', 'FPBI'!")
                end
            end
        end
        
        function [chosenInput, chosenSlacks, chosenParameters, chosenSolution] = chooseSolution(this, agent, inputs, slacks, front, paretoParameters)
            % Function to select a Pareto-optimal point on the Pareto front. The function stored in 
            % the ParetoController config or the function corresponding to a stored string is
            % called.
            
            if isa(this.config.metricFunction, 'function_handle')
                idx = this.config.metricFunction(front);
            else
                metricFunction = "select"+this.config.metricFunction;
                if ismethod(this, metricFunction)
                    idx = ParetoController.(metricFunction)(front);
                else
                    error("No function handle given for metric function! Try 'CUP', 'AEP', ATN or 'RoC'.")
                end
            end
            
            chosenSolution = front(idx,:);
            chosenInput = inputs{idx,1};
            chosenSlacks = slacks(idx,:);
            chosenParameters = paretoParameters(idx,:);
            agent.status.chosenWeights = paretoParameters(idx,:);
        end
        
        function [inputs, slacks, front, parameters] = determineFront(this, agent, optimizer, extremePoints,...
                inputsEP, slacksEP, parametersEP, chosenParameters, determinationScheme)
            % Function that calls the function handle or the function described by the stored string
            % to generate the Pareto front.
            
            if nargin < 9
                determinationScheme = this.config.frontDeterminationScheme;
            end
            if nargin < 8
                chosenParameters = [];
            end
            
            if isa(determinationScheme, 'function_handle')
                [inputs, slacks, front, parameters] = determinationScheme(this, agent, optimizer, extremePoints, chosenParameters);
            else
                FDS = "determine"+this.config.frontDeterminationScheme;
                if ismethod(this, FDS)
                    [inputs, slacks, front, parameters] = ParetoController.(FDS)(this, agent, optimizer, extremePoints, chosenParameters);
                else
                    error("No function handle given for metric function! Try 'AWDS', 'NBI' or 'FPBI'.")
                end
            end
            inputs = [inputsEP; inputs];
            slacks = [slacksEP; slacks];
            parameters = [parametersEP; parameters];
            this.status.front = front;
        end
        
        function [extremePoints, inputsEP, slacksEP, parametersEP] = determineExtremePoints(this, optimizeConstraints, costExpressions, agent)
            % Function that calls the function handle or the function described by the stored string
            % to calculate the optimization problem's extreme points.
            
            if isa(this.config.extremePointFunction, 'function_handle')
                [extremePoints, inputsEP, slacksEP, parametersEP] = this.config.extremePointFunction(this, optimizeConstraints, costExpressions, agent);
            else
                initializationMethod = "initialize"+this.config.extremePointFunction;
                if ismethod(this, initializationMethod)
                    [extremePoints, inputsEP, slacksEP, parametersEP] = ParetoController.(initializationMethod)(this, optimizeConstraints, costExpressions, agent);
                else
                    error("No function handle given for metric function! Try 'Lex', 'MWA' or 'MWAN'.")
                end
            end
        end
        
    end
    
    methods (Access=protected)
        function clear(this)
            this.evaluatedPoints = [];
        end
    end
    
    methods (Static)
        
        % optimizer preparation functions
        optimizer = prepareAWDS(paretoObj, optimizeConstraints, costExpressions, agent, ~)
        optimizer = prepareNBI(paretoObj, optimizeConstraints, costExpressions, agent, extremePoints)
        optimizer = prepareFPBI(paretoObj, optimizeConstraints, costExpressions, agent, extremePoints)
        optimizer = prepareWS(paretoObj, optimizeConstraints, costExpressions, agent, utopia, nadir, additionalSymbols, additionalCostExpression)
        
        % extreme point functions
        [extremePoints, inputsEP, slacksEP, parametersEP] = initializeMWA(paretoObj, optimizeConstraints, costExpressions, agent);
        [extremePoints, inputsEP, slacksEP, parametersEP] = initializeMWAN(paretoObj, optimizeConstraints, costExpressions, agent);
        [extremePoints, inputsEP, slacksEP, parametersEP] = initializeLex(paretoObj, optimizeConstraints, costExpressions, agent);
        
        % front evaluation functions
        [inputs, slacks, front, parameters] = determineAWDS(this, agent, optimizer, extremePoints, chosenParameters);
        [inputs, slacks, front, parameters] = determineNBI(this, agent, optimizer, extremePoints, chosenParameters);
        [inputs, slacks, front, parameters] = determineFPBI(this, agent, optimizer, extremePoints, chosenParameters);
        
        % metric functions
        [idx,utility] = selectCUP(varargin);
        [idx,utility] = selectAEP(varargin);
        [idx,utility] = selectRoC(varargin);
        [idx,utility] = selectATN(varargin);
        
        function config = getDefaultConfig()
            % TODO: sortieren
            config = struct;
            config.drMin = 0.2;
            config.distance2AllMin = [];
            config.interactivity = false;
            config.plotLabels = {};
            config.preperationMethod = 'auto';
            config.extremePointFunction = 'MWAN';
            config.metricFunction = 'CUP';
            config.frontDeterminationScheme = 'FPBI';
            
            config.getKneeRegion = false;
            config.AWDSrefinement = 0.4;
            config.normalization = 'dynamic';
            config.fixedNormValues = [];
            config.m = 20; % Resolution for FPBI
            config.focusPoint = 0; % FocusPoint is utopia point
            config.checkRedundancy = false;
            config.redundantCorrelation = 0.9; % threshold for correlation over that two objectives correlate
            config.independantCorrelation = 0; % threshold for correlation, if all correlation coefficients lie under this threshold
            config.corrUniqueThreshold = 1e-2;
            config.ignoreInPareto = [];
            config.redundantWeight = 5e-4;
            config.indepWeight = 1e-3;
            config.MWAweights = 5e-4;
            config.minRatioWeights = 1e-5;
            config.distanceInBP = 0.1;
            config.focusPoint = 0;
            config.secondaryL = 0.9; % Length in secondary directions is config.secondaryL*L
            config.diffTolBound = 1e-4; % Min distance between 2 points to be considered different points in paretoFilter
        end
    end
    
    methods (Static, Access = public)
        % more efficient implementation of setdiff for integers
        C = paretoSetDiff(A,B);
        
        % filter to check for weakly Pareto-optimal points
        idc = paretoFilter(paretoObj, potentialPts, fixedPointIdc);
        
        % function for checking cost functions for redundancy
        redundancyCheck(paretoObj);
        
        % find the outer points of the Pareto front
        borderPoints = getBorderPoints(pf);
        
        % n-dimensional cross product
        vec = crossn(RowVectorMatrix);
        
        % normalization using the utopia and nadir point
        normalizedPoints = ParetoNormalization(points, paretoObj);
        
        % find the adjacent points to the examined point 
        adjacentPts = getAdjacentPoints(pf, examinedPoint, ignorePoints);
    end
end

