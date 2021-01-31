classdef ParetoPlot < Figure
    % Subclass from Figure for pareto plots
    
    properties
        paretoFronts = {}; 
        dependsOnAgents = {};
        subplotDependsOnAgents = {};
        agentHandles = {}; 
    end
    properties (Access = private) 
        subplots 
        subplotIsCumulative = []; 
        subplotNormalized = [1]; 
        subplotLegendPrinted = [];
        linesToMakeTransparent = {}; 
    end
    methods
        % Constructor
        function obj = ParetoPlot(name, nRows, nColumns, varargin)
            obj@Figure(name, nRows, nColumns, varargin);
            obj.subplots = cell(1, obj.nRows*obj.nColumns);
            obj.linesToMakeTransparent     = cell(1, obj.nRows*obj.nColumns);
            obj.subplotDependsOnAgents      = cell(1, obj.nRows*obj.nColumns);
            obj.subplotLegendPrinted        = zeros(1, obj.nRows*obj.nColumns);
        end
               
        
        function addParetoFront(obj, agent, objectives, subplotIndex, cumulative,  ... 
                        varargin)
%                     normalized, showLegend, optionsFront, optionsSolution, 
        % ADDLINE adds settings for Pareto front, which shall be plotted, to the class
        % property paretoFronts and updates dependsOnAgents property if necessary
        % Input:
        %   agent           agent handle from which to take variables 
        %   objectives      which objectives to take (indexes or names)
        %   subPlotIndex    index of subplot to plot into
        %   isCumulative    whether to plot all fronts cumulative 
        %   normalized      whether or not to normalize fronts [0]
        %   showLegend      true/ [false] show legend
        %   optionsFront    cell array with plot (line) options for front
        %   optionsSolution cell array with scatter options for selected
        %   solution
            normalized = 1; 
            showLegend = 0; 
            optionsFront = {};
            optionsSolution = {}; 
            
            if nargin >= 6 && ~isempty(varargin{1})
                normalized = varargin{1}; 
            end            
            if nargin >= 7 && ~isempty(varargin{2})
                showLegend = varargin{2}; 
            end
            if nargin >= 8 && ~isempty(varargin{3})
                optionsFront = varargin{3}; 
            end
                        
            if nargin >= 9 && ~isempty(varargin{4})
                optionsSolution = varargin{4}; 
            end
            
            if numel(objectives) > 2 && cumulative
                warning('PARODIS: ParetoPlot: addParetoFront: Cumultive plots only for 2 objectives supported. Will be ignored.')
                cumulative = 0; 
            end
            
            % set default options if not given
            if cumulative
                optionsFront    = obj.addDefaultOption(optionsFront,    {'LineStyle', '-', 'Marker', 'none', 'Color', [0.29688,0.53125,0.99609]});
                optionsSolution = obj.addDefaultOption(optionsSolution, {'Marker', 'o', ... 
                                                                         'MarkerEdgeColor', [0.91797,0.50781,0.17188], ...
                                                                         });
            else 
                optionsFront    = obj.addDefaultOption(optionsFront,    {'LineStyle', 'none', 'Marker', 'x', 'Color', [0.29688,0.53125,0.99609]});
                optionsSolution = obj.addDefaultOption(optionsSolution, {'Marker', 'o', ...
                                                                         'MarkerEdgeColor', [0.91797,0.50781,0.17188], ...
                                                                        });
            end
            
            pf = struct;
            pf.agent = agent; 
            pf.objectives = objectives; 
            pf.cumulative = cumulative; 
            pf.normalized = normalized; 
            pf.showLegend = showLegend; 
            pf.optionsFront = optionsFront; 
            pf.optionsSolution = optionsSolution; 
            
            if ~isempty(obj.subplots{subplotIndex})
                warning('PARODIS: ParetoPlot: addParetoFront: Subplot %d already contained a Pareto front, which has now been overwritten!', subplotIndex)
            end
            obj.subplots{subplotIndex} = pf;
                        
            % Add agent name to dependsOnAgents property if necessary
            if ~any(strcmp(obj.dependsOnAgents, agent.name))
                obj.dependsOnAgents{end+1} = agent.name;
            end
                        
        end
        
        function optionsCell = addDefaultOption(obj, optionsCell, defaultOptions)
        % helper function to add default options to cell array    
            for ii = 1:2:size(defaultOptions, 2)
                if ~any( strcmp(optionsCell, defaultOptions{ii}) )
                    optionsCell{end+1} = defaultOptions{ii};
                    optionsCell{end+1} = defaultOptions{ii+1};                    
                end
            end
            
        end % end addDefaultOption()

        
        function show(obj, callingAgent, isFinalUpdate)
        % SHOW uses Figure-class implemenation fo show() to set fighandle.Visible to 'on'
        % and then does the actual plotting, i.e. all paretoFronts 
            
            % if entire instance does not depend on callingAgent, nothing needs to
            % be done
            if ~any(strcmp(obj.dependsOnAgents, callingAgent.name))
                return
            end
            
            % Turn on only for debugging!
%             show@Figure(obj);
            
            % Go through subplots  
            for jj = 1 : size(obj.subplotAxisHandles, 2)
                pf = obj.subplots{jj}; 

                % Skip if not dependent on callingAgent !
                if isempty(pf) || ~strcmp(pf.agent.name, callingAgent.name)                    
                    continue
                end
                
                                
                % check which objectives to use 
                [takeObj, objInd] = obj.checkObjectives(pf);
                
                % stop if only 1 competing obj was found
                if numel(objInd) < 2
                    warning('PARODIS: ParetoPlot: show: Cannot plot a Pareto front for less than 2 objectives!')
                    continue
                end

                % Do the actual plotting

                % if cumulative, completely separate
                if pf.cumulative == 1
                    
                    if any(sort(pf.objectives) ~= sort(objInd))
                        warning('PARODIS: ParetoPlot: show: Cumulative plot omitted because one of the objectives was redundant this time')
                        continue
                    end
                    grid(obj.subplotAxisHandles{jj}, 'on'); 
                    
                    % transparency values for cumulative plots
                    transFront    = max(0.2, 1-callingAgent.simulation.k_total * 0.02);
                    transSolution = max(0.3, 1-callingAgent.simulation.k_total * 0.01);
                    
                    
                    if obj.plotLive && callingAgent.simulation.config.livePlot % if live plotting is / was enabled
                        % read in Data
                        [front, solution] = obj.readData(pf, objInd, isFinalUpdate);
                        
                        hold(obj.subplotAxisHandles{jj}, 'on');
                        
                        % First make last line transparent (if there is
                        % any)
                        if ~isempty(obj.linesToMakeTransparent{jj})
                            obj.linesToMakeTransparent{jj}.haxFront.Color = [haxFront.Color transFront];
                            obj.linesToMakeTransparent{jj}.haxSolution.MarkerEdgeAlpha = transSolution;                            
                            
                        end
                        
                        % Unless this is the final update, plot new line
                        % (not transparent)
                        if ~isFinalUpdate
                            % reset lines to make transparent
                            obj.linesToMakeTransparent = [];
                            
                            % plot (solid) new line+solution from current time step
                            if ~obj.subplotLegendPrinted(jj)
                                haxFront = plot(obj.subplotAxisHandles{jj}, front(:,1), front(:,2), pf.optionsFront{:}); %, pf.optionsFront);
                                haxSolution = scatter(obj.subplotAxisHandles{jj}, solution(1), solution(2), pf.optionsSolution{:});
                            else % surpress legend if already printed                                
                                haxFront = plot(obj.subplotAxisHandles{jj}, front(:,1), front(:,2), pf.optionsFront{:}, 'HandleVisibility', 'off'); %, pf.optionsFront);
                                haxSolution = scatter(obj.subplotAxisHandles{jj}, solution(1), solution(2), pf.optionsSolution{:}, 'HandleVisibility', 'off');                                
                            end
                            % and add it to make transparent
                            obj.linesToMakeTransparent{jj}.haxFront = haxFront;
                            obj.linesToMakeTransparent{jj}.haxSolution = haxSolution;
                            
                            % print legend
                            obj.printLegend(pf, jj, 1)
                        end
                        
                        
                        
                    else %~obj.plotLive  -> cumulative, but non-live
                    % -> we have to plot every front in a loop
                        hold(obj.subplotAxisHandles{jj}, 'on');
                        for i = 1:length(pf.agent.history.pareto.fronts)
                            
                            [front, solution] = obj.readDataFromHistory( pf, i, objInd );
                            
                            haxFront    = plot(obj.subplotAxisHandles{jj},  front(:, 1), front(:, 2), pf.optionsFront{:} );
                            haxSolution = scatter(obj.subplotAxisHandles{jj},  solution(1), solution(2), pf.optionsSolution{:} );
                            
                            % Make transparent
                            haxFront.Color = [haxFront.Color transFront];
                            haxSolution.MarkerEdgeAlpha = transSolution;
                            
                        end
                        
                        % print legend, but temporarily suppress warning of extra
                        % entries
                        warning('off', 'MATLAB:legend:IgnoringExtraEntries');
                        obj.printLegend(pf, jj);
                        warning('on', 'MATLAB:legend:IgnoringExtraEntries');
                    end
                    
                else % non-cumulative plot
                    [front, solution] = obj.readData(pf, objInd, isFinalUpdate);
                    
                    % delete axes 
                    cla(obj.subplotAxisHandles{jj});
                    
                    if numel(objInd) == 2
                        hold(obj.subplotAxisHandles{jj}, 'on');                        
                        haxFront = plot(obj.subplotAxisHandles{jj}, front(:,1), front(:,2), pf.optionsFront{:}); 
                        haxSolution = scatter(obj.subplotAxisHandles{jj}, solution(1), solution(2), pf.optionsSolution{:});
                    else % 3 objectives
                        hold(obj.subplotAxisHandles{jj}, 'on');
                        haxFront = plot3(obj.subplotAxisHandles{jj}, front(:,1), front(:,2), front(:,3), pf.optionsFront{:}); 
                        hold(obj.subplotAxisHandles{jj}, 'on');
                        
                        haxSolution = scatter(obj.subplotAxisHandles{jj}, solution(1), solution(2), solution(3), pf.optionsSolution{:});
                        
                        set(obj.subplotAxisHandles{jj},'Xdir','reverse');
                        set(obj.subplotAxisHandles{jj},'Ydir','reverse');
                    end
                    grid(obj.subplotAxisHandles{jj}, 'on'); 
                    
                    % print legend
                    if pf.showLegend                   
                        legend(obj.subplotAxisHandles{jj}, 'Pareto Front', 'Chosen Solution');
                    end
                end
                
                % set labels (same for all cases)
                obj.setLabels(pf, objInd, obj.subplotAxisHandles{jj})
                    
                 
            end % end for loop        

            % Show figure at the very end only!
            drawnow;
            show@Figure(obj);  
        end % end show()
        
        function [] = printLegend(obj, pf, jj, onlyOnce)
        % print legend if pf.showLegend && either always or, if onlyOnce is set, only if has not been printed yet    
            if nargin == 3 
                onlyOnce = 0; 
            end
            
            if pf.showLegend && ~onlyOnce || pf.showLegend && ( onlyOnce && ~obj.subplotLegendPrinted(jj) ) 
                legend(obj.subplotAxisHandles{jj}, 'Pareto Front', 'Chosen Solution');
                obj.subplotLegendPrinted(jj) = 1;
            end
        end
        
        function [] = applyOptions(obj, pf, haxFront, haxSolution)
        % NOT USED ANYMORE!
        % apply options from pf.optionsFront and pf.optionsSolution, if
        % handles are given
            
            if ~isempty(haxFront)
                % Set Options Front
                for ii = 1:2:size(pf.optionsFront, 2)
                    haxFront.(pf.optionsFront{ii}) = pf.optionsFront{ii+1};
                end
            end
            if nargin == 4 && ~isempty(haxSolution)
                % Set Options Solution
                for ii = 1:2:size(pf.optionsSolution, 2)
                    haxSolution.(pf.optionsSolution{ii}) = pf.optionsSolution{ii+1};
                end
            end
            
        end % end applyOptions()
        
        function [takeObj, objInd] = checkObjectives(obj, pf)
        % since evaluated objectives might change between steps, check
        % which have actually been evaluated
            
            takeObj = [];
            objInd = []; % we also need the row index of the objective in controller.status.front
            for ii = 1 : numel(pf.objectives)
                if any(pf.objectives(ii) == pf.agent.controller.status.conflictingObj)
                    if numel(takeObj) < 3
                        takeObj(end+1) = pf.objectives(ii);
                        objInd(end+1) = find(pf.objectives(ii) == pf.agent.controller.status.conflictingObj);
                    else % we can plot only 3 at onc
                        warning('PARODIS: ParetoPlot: show: only 3 objectives will be plotted!')
                        continue
                    end
                end
            end
            
        end % end checkObjectives()
        
        function [front, solution] = readData(obj, pf, objInd, isFinalUpdate)
            
            % first get entire front / chosenSolution
            if ~isFinalUpdate % live
                frontAll    = pf.agent.controller.status.front;
                solutionAll = pf.agent.status.pareto.chosenSolution;
            else % final plot
                frontAll    = pf.agent.history.pareto.fronts{end};
                solutionAll = pf.agent.history.pareto.chosenSolutions(end, :);                
            end
            
            % second normalize if necessary
            if pf.normalized
                frontAll = pf.agent.controller.ParetoNormalization(frontAll, pf.agent.controller);
                solutionAll = pf.agent.controller.ParetoNormalization(solutionAll, pf.agent.controller);
            end
            
            % now pick only the objectives we need
            front    = obj.pickDataObjOnly(frontAll, objInd);
            solution = obj.pickDataObjOnly(solutionAll, objInd);
            
            % if only 2-dimension, sort it 
            if numel(objInd) == 2
                [front(:,1), I] = sort(front(:,1));
                front(:,2) = front(I,2);
            end
        end % end readData()
        
        function [front, solution] = readDataFromHistory(obj, pf, i, objInd )
                            
            frontAll    = pf.agent.history.pareto.fronts{i}; 
            solutionAll = pf.agent.history.pareto.chosenSolutions(i, :); 

            % second normalize if necessary
            if pf.normalized
                % Attention: Here, we use the 'simple' normalization
                % method, i.e. not the *true* Utopia point 
                temp = pf.agent.controller.ParetoNormalization([frontAll; solutionAll]);
                frontAll    = temp(1:end-1, :); 
                solutionAll = temp(end,:);
            end
                        
            % now pick only the objectives we need
            front    = obj.pickDataObjOnly(frontAll, objInd);
            solution = obj.pickDataObjOnly(solutionAll, objInd);
            
            % if only 2-dimension, sort it 
            if numel(objInd) == 2
                [front(:,1), I] = sort(front(:,1));
                front(:,2) = front(I,2);
            end
        end % end readDataFrontFromHistory()
        
        function dataObj = pickDataObjOnly(obj, data, objInd) 
        % helper function
            for ii = 1 : numel(objInd)
                dataObj(:,ii) = data(:, objInd(ii));
            end
                        
        end % end pickDataObjOnly()
        
        
        function [] = setLabels(obj, pf, objInd, subplotAxisHandle)
        % set label names    
            % iterate through cost fun names and find the right one
            fn = fieldnames(pf.agent.controller.costFunctionIndexes);
            for ii = 1 : numel(objInd)
                
                for k=1:numel(fn)
                    if pf.agent.controller.costFunctionIndexes.(fn{k}) == objInd(ii)
                        labelNames{ii} = fn{k};
                        continue
                    end
                end
                
            end
            xlabel(subplotAxisHandle, labelNames(objInd(1)), 'Interpreter','none' );
            ylabel(subplotAxisHandle, labelNames(objInd(2)), 'Interpreter','none' );
            if numel(objInd) == 3
                zlabel(subplotAxisHandle, labelNames(objInd(3)), 'Interpreter','none' );
            end
        end
        
    end % end methods
end
