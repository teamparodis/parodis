function [extremePoints, inputsEP, slacksEP, weightsEP] = initializeLex(paretoObj, optimizeConstraints, costExpressions, agent)
% Lexicographic approach for finding extreme points on the pareto front.
% Minimize f1 without regard of the others, then minimize the f2, s.t.
% f1 <= f1_min

persistent sl

dim = numel(costExpressions);
n = numel(paretoObj.status.conflictingObj);
extremePoints = zeros(n); % array with extreme points

inputsEP = cell(n,1);
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

% call all objectives
for objective = 1:n
    additionalConstraints = [sl >= 0];
    lastCostFcnValues = [];
    
    % call all other objectives in lexicographic order
    for order = costFcnOrder(objective,:)
        
        fullCostExpression = costExpressions{order} + sl * ones(dim, 1) * 1e5;
        diagnostics = optimize([optimizeConstraints; additionalConstraints], fullCostExpression,  agent.controller.yalmipOptions);
        
        if diagnostics.problem ~= 0
            error("YALMIP error " + diagnostics.problem + " detected: " + yalmiperror(diagnostics.problem));
        end
        
        costFcnValues = value([costExpressions{paretoObj.status.conflictingObj}]);
        if ~isempty(lastCostFcnValues) && sum(abs(costFcnValues - lastCostFcnValues)) <= 1e-10 % if values stop changing end lexicographic order
            break;
        end
        
        lastCostFcnValues = costFcnValues;
        % additional constraints on calculated objectives
        if costFcnValues(order) ~= 0
            additionalConstraints = [additionalConstraints; costExpressions{order}/...
                costFcnValues(order) <= 1 + sl(order)];
        else
            additionalConstraints = [additionalConstraints; costExpressions{order}...
             <= sl(order)];
        end
    end
    
    extremePoints(objective,:) = costFcnValues;
    inputsEP{objective,1} = value(agent.model.u);
    
    slackVariableNames = fieldnames(agent.controller.slackVariables);
    fillSlacks = struct;
    for idx=1:length(slackVariableNames)
        fillSlacks.(slackVariableNames{idx}) = value(agent.controller.slackVariables.(slackVariableNames{idx}));
    end
    
    slacksEP(objective,1) = fillSlacks;
end

% check for double EP
[extremePoints, uniqueIdc] = uniquetol(extremePoints, 1e-10, 'ByRows', 1);
weightsEP     = weightsEP(uniqueIdc,:);
inputsEP      = inputsEP(uniqueIdc);
slacksEP      = slacksEP(uniqueIdc);

paretoObj.status.utopia = min(extremePoints);
paretoObj.status.nadir = max(extremePoints);

if size(extremePoints,1) > 1
    filteredFront = ParetoController.paretoFilter(paretoObj, extremePoints, []);
    extremePoints = extremePoints(filteredFront,:);
    weightsEP     = weightsEP(filteredFront,:);
    inputsEP      = inputsEP(filteredFront);
    slacksEP      = slacksEP(filteredFront);
end
end

