function [inputs, weights] = coneInit(paretoObj)
%CONEMETHOD Summary of this function goes here
%   Detailed explanation goes here

persistent z f

if isempty(z)
    z = sdpvar(1);
end

dim = numel(paretoObj.agent.controller.costFunctions);

if isempty(f)
    f = sdpvar(1,dim);
end

x0 = paretoObj.agent.history.x(:,end);
d = paretoObj.agent.status.dPred;

[inputs, weights] = Pareto.Lexicographic(paretoObj);
paretoObj.evaluatedPoints = [];

m = paretoObj.agent.config.pareto.mInit;

paretoObj.status.evaluatedBeta = [];
betaVec = [];

for theta=0:pi/(2*m):pi/2
    betaTemp = getRadialPoints(theta,1,m, paretoObj);
    betaVec = [betaVec; betaTemp];
end

UPNPdiff = paretoObj.agent.history.pareto.nadirs(end,:) - paretoObj.agent.history.pareto.utopias(end,:);

for i=1:numel(paretoObj.agent.controller.costFunctions)
    f(i) = paretoObj.agent.controller.costExpressions{i};
end

for border = 1:size(betaVec,1)
    
    constr = [z >= 0; z*betaVec(border,:) >= (f-paretoObj.agent.history.pareto.utopias(end,:))./UPNPdiff];%./paretoObj.agent.history.pareto.nadirs(end,:) >= 0];
    
    [u, slack, feasibilityCode] = paretoObj.agent.controller.getInput(zeros(1,dim), x0, paretoObj.agent, constr,-paretoObj.agent.controller.costExpression+z);%
%         [u, slack, feasibilityCode] = paretoObj.agent.controller.getInput(zeros(1,dim), x0, paretoObj.agent, [], ...
%         [], 'BI', paretoObj.agent.history.pareto.utopias(end,:),paretoObj.agent.history.pareto.nadirs(end,:),...
%         zeros(1,dim),-betaVec(border,:));
    
    if feasibilityCode ~= 0
        continue
    end
    
    x = paretoObj.agent.predictTrajectory(u, d, x0);
    params = paretoObj.agent.status.paramValues;
    Ns = paretoObj.agent.controller.numScenarios;
    
    for ii = 1:numel(paretoObj.agent.controller.costFunctions)
        J_opt(1,ii) = paretoObj.agent.controller.costFunctions{ii}.buildExpression(x,u,d,params,Ns,slack,paretoObj.agent.config.T_s);
    end
%     solver_out = optimize(constr,-z, paretoObj.agent.controller.yalmipOptions);
    
    if ismember(J_opt,[0 0 0],'row')
        continue
    end
    
    if isempty(fields(paretoObj.slackValues))
        paretoObj.slackValues = slack;
    else
        paretoObj.slackValues(end+1) = slack;
    end
    
    paretoObj.evaluatedPoints = [paretoObj.evaluatedPoints; J_opt];
    
    inputs = [inputs; {u}];
    weights = [weights; betaVec];
    
end

paretoObj.agent.history.pareto.nadirs(end,:) = max(paretoObj.evaluatedPoints);

keepPoints = paretoFilter(Pareto.ParetoNormalization( paretoObj.evaluatedPoints, paretoObj ));
possibleEP = paretoObj.evaluatedPoints(keepPoints,:);
% [~,EPpos] = min(possibleEP);
% keepPoints = keepPoints(EPpos);
paretoObj.evaluatedPoints = paretoObj.evaluatedPoints(keepPoints,:);
weights = weights(keepPoints,:);
inputs = inputs(keepPoints);

paretoObj.slackValues = paretoObj.slackValues(keepPoints);
paretoObj.agent.history.pareto.utopias(end,:) = min(possibleEP);
paretoObj.agent.history.pareto.nadirs(end,:) = max(possibleEP);

end

%%
function [betaVec] = getRadialPoints(theta,k_last,m,paretoObj)

k_now = k_last+1;
m_i = m;%round(m*prod(sin(theta)));
if k_now == numel(paretoObj.agent.controller.costFunctions)
    betaVec = cos(theta(1));
    for i = 2:length(theta)
        betaVec = [betaVec, cos(theta(i))*prod(sin(theta(1:i-1)))];
    end
    betaVec = [betaVec, prod(sin(theta))];
    
    if all(betaVec >= 1e-12) || (~isempty(paretoObj.status.evaluatedBeta) &&...
            ismember(betaVec, paretoObj.status.evaluatedBeta,'rows'))
        betaVec = [];
        return;
    end
    
    paretoObj.status.evaluatedBeta = betaVec;
    
else
    betaVec = [];
    for theta_i = 0:pi/(2*m_i):pi/2
        [betaTemp] = getRadialPoints([theta, theta_i],k_now,m, paretoObj);
        betaVec = [betaVec; betaTemp];
    end
end

end

%%

function idx = paretoFilter(potentialPts)
idxPO = [];
for i=1:size(potentialPts,1)
    diff = potentialPts([1:i-1,i+1:end],:)-potentialPts(i,:);
    diff(diff > -1e-5 & diff < 1e-5) = 0;
    s = sign(diff);
    if any(all(s>=0,2) & any(s==1,2))
        continue;
    end
    idxPO = [idxPO, i];
end

[~,idxUnique] = uniquetol(potentialPts, 1e-5, 'ByRows', true);

idx = intersect(idxUnique, idxPO);

end