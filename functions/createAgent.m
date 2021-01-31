function [agent] = createAgent(name, ansatzHandle, x0, implicitPrediction)
% createAgent   Helper function for instantiating agent directly from ansatz
%   
    if nargin < 4
        implicitPrediction = false;
    end
    
	[model, controller, T_s] = ansatzHandle(implicitPrediction);
	agent = Agent(name, model, controller, T_s, x0);

end