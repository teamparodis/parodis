function optimizerAWDS = prepareAWDS(paretoObj, optimizeConstraints, costExpressions, agent, ~)
%Generate the optimizer for AWDS using the prepareWS method
optimizerAWDS = ParetoController.prepareWS(paretoObj, optimizeConstraints, costExpressions, agent);
end


