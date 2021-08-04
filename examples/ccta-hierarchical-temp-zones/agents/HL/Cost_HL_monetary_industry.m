classdef Cost_HL_monetary_industry
    %MONETARYCOSTFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function slacks = getSlacks(model, agent, params)
            slacks = struct;
            slacks.P_pos = sdpvar( length(agent.config.T_s), 1 );
            slacks.peak_diff = sdpvar;
            
        end
        
        function constraints = getConstraints(model, agent, slacks, params)
            P_grid = model.u(1, :)';
            
            constraints = [ 
                (0 <= slacks.P_pos <= 1e5):'artifical bound on P_pos';
                ( P_grid <= slacks.P_pos ):'slack variable to Pgrid into + and -';
                ( slacks.peak_diff >= P_grid - params.Pgrid_max{1} ):'epigraph of max(0, max(Pgrid - Pgrid_max))';
                ( slacks.peak_diff >= 0 ):'epigraph of max(0, max(Pgrid - Pgrid_max))';
            ];
        end
        
        function [slacks, constraints] = getSlacksAndConstraints(model, agent, params)
            
        end
        
        function expr = buildExpression(x, u, d, params, Ns, slacks, T_s)
            expr = 0;
            for s=1:Ns
                expr = expr + Cost_HL_monetary_industry.buildExpressionSingleScen(x{s}, u, d{s}, extractScenario(params, s), slacks, T_s);
            end
        end
        
        function [exprSingle] = buildExpressionSingleScen(x, u, d, params, slacks, T_s)
            
            P_grid = u(1, :)';
            P_chp = u(2, :)';
            Q_heat = u(3, :)';
            
            % peak costs
            
            % to avoid nesting max terms, yalmip does not like that
            l_peakpun = params.peak_cost_factor * slacks.peak_diff;
            
            % buying costs
            l_buy = T_s/60 .*params.price_pred * slacks.P_pos;
            
            % selling profits
            l_sell = params.c_grid_sell * (T_s/60) *(P_grid - slacks.P_pos);
            
            % CHP
            c_gas = 0.045; % 4.5 cent per kWh gas
            eta_chp = 0.892; %from data sheet
            c_chp = 0.677; % Stromkennzahl

            l_chp = (c_gas*(1+1/c_chp)/eta_chp)*(T_s/60) *P_chp;

            %NEW: cost term for gas heating. Assumed 97% efficiency and 4.5 ct/kWh
            %gas energy
            l_Qheat = (1/0.97)*c_gas*(T_s/60) *Q_heat;

            % sum all costs and scale them to the step width, as costs are per kWh
            l_mon = ( l_buy + l_sell  + l_chp + l_Qheat);
            
            exprSingle = l_mon + l_peakpun;
        end
        
        function [horizon] = evaluateHorizon(x, u, d, params, slacks, T_s)
            P_grid = u(1, :);
            P_chp = u(2, :);
            Q_heat = u(3, :);
            
            % CHP
            c_gas = 0.045; % 4.5 cent per kWh gas
            eta_chp = 0.892; %from data sheet
            c_chp = 0.677; % Stromkennzahl
            
            l_buy   = (T_s/60).*params.price_pred{1} .* max(0, P_grid);
            l_sell  = (T_s/60).*params.c_grid_sell{1} .* min(0, P_grid);
            l_chp   = (c_gas*(1+1/c_chp)/eta_chp)*(T_s/60) .* P_chp;
            l_Qheat = (1/0.97)*c_gas*(T_s/60) .* Q_heat;
            
            % build trajectory of peak cost progression over horizon
            peak_diff = max(0, P_grid - params.Pgrid_max{1});
            peak_traj = cummax(peak_diff);
            peak_traj(peak_traj < 0) = 0; % at the beginning Peaks might be < 0 if P_grid was <0
            peak_vector = (peak_traj(1,:) - [0, peak_traj(1, 1:end-1)]).*1;

            l_peakpun = params.peak_cost_factor{1} * peak_vector;
            
            l_mon = l_buy + l_sell  + l_chp + l_Qheat;
            horizon = l_mon + l_peakpun;
        end
    end
end

