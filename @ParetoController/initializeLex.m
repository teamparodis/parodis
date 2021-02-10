function [extremePoints, inputsEP, slacksEP, weightsEP] = initializeLex(paretoObj, optimizeConstraints, costExpressions, agent)
% Lexicographic approach for finding extreme points on the pareto front.
% Minimize f1 without regard of the others, then minimize the f2, s.t.
% f1 <= f1_min

persistent sl

if agent.config.debugMode
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
else
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
end

options = [options, agent.config.solverOptions];
yalmipOptions = sdpsettings( options{:} );

dim = numel(costExpressions);
n = numel(paretoObj.status.conflictingObj);
extremePoints = zeros(n); % array with extreme points

inputsEP = cell(n,1);
slacksEP = cell(n,1);
weightsEP = eye(n); % array with weights as row vector, only weighting one objective

costFcnOrderIdc = zeros(n); % order in which objective functions are used

for i = 0:n-1
    costFcnOrderIdc(i+1,:) = (1+i):(n+i);
end
costFcnOrderIdc(costFcnOrderIdc > n) = costFcnOrderIdc(costFcnOrderIdc > n) - n;
% results in [1 2 3; 2 3 1; 3 1 2]

costFcnOrder = paretoObj.status.conflictingObj(costFcnOrderIdc);

if isempty(sl)
    sl = sdpvar(1,dim);
end

for objective = 1:n
    additionalConstraints = [];
    lastCostFcnValues = [];
    
    for order = costFcnOrder(objective,:)
        
        fullCostExpression = costExpressions{order} + sl*sl'*1e5;
        diagnostics = optimize([optimizeConstraints; additionalConstraints], fullCostExpression, yalmipOptions);
        
        if diagnostics.problem ~= 0
            error("YALMIP error " + diagnostics.problem + " detected: " + yalmiperror(diagnostics.problem));
        end
        
        costFcnValues = value([costExpressions{paretoObj.status.conflictingObj}]);
        if isequal(costFcnValues, lastCostFcnValues) % if values stop changing end lexicographic order
            break;
        end
        
        lastCostFcnValues = costFcnValues;
        additionalConstraints = [additionalConstraints; costExpressions{order}/costFcnValues(order) <= 1 + sl(order)];
    end
    
    extremePoints(objective,:) = costFcnValues;
    inputsEP{objective,1} = value(agent.model.u);
    
    slackVariableNames = fieldnames(agent.controller.slackVariables);
    fillSlacks = struct;
    for idx=1:length(slackVariableNames)
        fillSlacks.(slackVariableNames{idx}) = value(agent.controller.slackVariables.(slackVariableNames{idx}));
    end
    
    slacksEP{objective,1} = fillSlacks;
end

paretoObj.status.utopia = min(extremePoints);
paretoObj.status.nadir = max(extremePoints);

end

