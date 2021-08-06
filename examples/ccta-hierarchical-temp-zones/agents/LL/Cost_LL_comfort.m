classdef Cost_LL_comfort < CostFunction
    %COMFORTCOSTFUNCTION Cost function for temperature deviation of lower level model     %   Detailed explanation goes here
    
    methods (Static)

        function [slacks] = getSlacks(model, agent, params) 
            slacks = struct;
            slacks.theta_aux = sdpvar(size(model.x{1},1), size(model.x{1},2), 'full' ) ;
        end
        
        function [constraints] = getConstraints(model, agent, slacks, params)
            constraints = (slacks.theta_aux == model.x{1} - params.theta_ref{1}):'auxiliary slack variable for (theta - theta_ref)';
        end
        
        function expr = buildExpression(x, u, d, params, Ns, slacks, T_s)
            T_s_extended = [T_s mean(T_s)];
            
            n_tempZones = size(x{1}, 1);
            % Weights for each temperature zone proportional to thermal Capacity! and
            % such that sum(w_th) == 1
            w_th = [0.1288    0.2658    0.1196    0.0579    0.1842    0.1842    0.0555    0.0013    0.0027];
            
            % exprSingle =  numel(T_s_extended) * (T_s_extended'./sum(T_s_extended)  .* slacks.theta_aux)' * slacks.theta_aux;
            expr = 0;
            for ii = 1 : n_tempZones
                expr = expr + w_th(ii) * T_s_extended .* slacks.theta_aux(ii,:) * slacks.theta_aux(ii,:)';
            end
        end
        
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            w_th = [0.1288    0.2658    0.1196    0.0579    0.1842    0.1842    0.0555    0.0013    0.0027];
            w_th_mat = repmat(w_th', 1, length(T_s)); 
            T_s_mat = repmat(T_s, 9, 1); 
                      
            horizon = w_th_mat .* T_s_mat .* (x{1}(:,1:end-1) - params.theta_ref{1}).^2; 
            horizon = sum(horizon); 
            
        end
    end
    
end

