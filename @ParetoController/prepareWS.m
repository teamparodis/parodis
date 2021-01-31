function optimizerWS = prepareWS(paretoObj, optimizeConstraints, costExpressions, agent, utopia, nadir, additionalSymbols, additionalCostExpression)
%prepareWS is a general function for defining optimizers for a weighted sum problem
% INPUTS:
%   paretoObj: ParetoController object
%   optimizeConstraints: constrints the system is subjected to
%   costExpressions: cell array of cost expressions
%   agent: Agent object the ParetoController corresponds to
%   OPTIONAL:
%       utopia: custom utopia point, if set to zeros(...) with nadir = ones(...) no normalization is
%           applied
%       nadir: custom nadir point, if set to ones(...) with utopia = zeros(...) no normalization is
%           applied
%       additionalSymbols: additional symbols that are used as additional inputs for the optimizer
%       additionalCostExpression: additional cost expressions that are added to the weighted sum

if agent.config.debugMode
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 1, 'verbose', 2,'convertconvexquad', 1,'showprogress', 1};
else
    options = {'solver', agent.config.solver, 'cachesolvers', 1, 'debug', 0, 'verbose', 0,'convertconvexquad', 1};
end

doOwnNormalization = false;

if nargin < 8
    additionalCostExpression = 0;
end

if nargin < 7
    additionalSymbols = [];
    doOwnNormalization = true;
end

if nargin < 6
    nadir = paretoObj.status.nadir;
end

if nargin < 5
    utopia = paretoObj.status.utopia;
end

options = [options, agent.config.solverOptions];
yalmipOptions = sdpsettings( options{:} );

% preset weights for independant and ignored Objectives
costExpressionWeights = paretoObj.weightSyms';
costExpressionWeights(paretoObj.status.independantObj) = paretoObj.config.indepWeight;
costExpressionWeights(paretoObj.status.redundantObj) = paretoObj.config.redundantWeight;
costExpressionWeights(paretoObj.config.ignoreInPareto) = paretoObj.defaultWeights(paretoObj.config.ignoreInPareto);

% build normalized cost expression
if doOwnNormalization
    WSCostExpression = ([costExpressions{:}]-utopia)./(nadir - utopia)*costExpressionWeights + additionalCostExpression;
else
   WSCostExpression = ParetoController.ParetoNormalization([costExpressions{:}],paretoObj)*costExpressionWeights + additionalCostExpression;
end

WSSymbols = [paretoObj.weightSyms(paretoObj.status.conflictingObj), additionalSymbols];

% define what optimizer should output
output = {agent.model.u};

% optimizer has to deliver all slack variables, since value()
% does not work with optimizer
slackVariableNames = fieldnames(agent.controller.slackVariables);
for idx=1:length(slackVariableNames)
    output{end+1} = agent.controller.slackVariables.(slackVariableNames{idx});
end

% build optimizer
optimizerWS = optimizer(optimizeConstraints, WSCostExpression, yalmipOptions,...
    WSSymbols, output);

end

