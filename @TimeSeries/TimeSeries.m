classdef TimeSeries < Figure
    % Subclass from Figure for actual plots:

    properties
        lineSeries = {};
        heatMaps = {};
        dependsOnAgents = {};
        subplotDependsOnAgents = {};
        agentHandles = {};
        doLinkAxes = 1;
    end
    properties (Access = private)
        subplotAxisPredictionTimesAndWidths = {};
        subplotFixedXLimits = {};
        subplotFixedYLimitsLeft = {};
        subplotFixedYLimitsRight = {};
        subplotTempXLimits = {};
        subplotHasRightY = [];
        subplotHasLineSeries = [];
        subplotHasHeatMap = [];
        subplotAxisPredictionBarAt = {};
        subplotHasLegend = [];
        linesToDelete = {};
        subplotLinesToDelete = {};
    end
    methods
        % Constructor
        function obj = TimeSeries(name, nRows, nColumns, varargin)
            obj@Figure(name, nRows, nColumns, varargin);
            obj.subplotAxisPredictionTimesAndWidths = cell(1, obj.nRows*obj.nColumns);
            obj.subplotFixedXLimits         = cell(1, obj.nRows*obj.nColumns);
            obj.subplotTempXLimits         = cell(1, obj.nRows*obj.nColumns);
            obj.subplotFixedYLimitsLeft     = cell(1, obj.nRows*obj.nColumns);
            obj.subplotFixedYLimitsRight    = cell(1, obj.nRows*obj.nColumns);
            obj.subplotHasRightY            = zeros(1, obj.nRows*obj.nColumns);
            obj.subplotHasLineSeries        = zeros(1, obj.nRows*obj.nColumns);
            obj.subplotHasHeatMap          = zeros(1, obj.nRows*obj.nColumns);
            obj.subplotAxisPredictionBarAt  = cell(1, obj.nRows*obj.nColumns);
            obj.subplotHasLegend            = zeros(1, obj.nRows*obj.nColumns);
            obj.subplotDependsOnAgents      = cell(1, obj.nRows*obj.nColumns);
            obj.subplotLinesToDelete        = cell(1, obj.nRows*obj.nColumns);
        end

        function setDoLinkAxes(obj, doLinkAxes)
            obj.doLinkAxes = doLinkAxes;
        end

        function setFixedXLimits(obj, subplotIndex, xLimits)
            obj.subplotFixedXLimits{subplotIndex} = xLimits;
        end

        function setFixedYLimits(obj, subplotIndex, yLimits, varargin)
        % Set fixed limits for both left or right y-axis.
        % If you want to set the limits for the right axis, the additional
        % argument must be 'right'. Otherwise, 'left' is used.
            if nargin == 4 && strcmp(varargin{1}, 'right')
                obj.subplotFixedYLimitsRight{subplotIndex} = yLimits;
            else
                obj.subplotFixedYLimitsLeft{subplotIndex} = yLimits;
            end
        end

        function addLine(obj, agent, variable, variableIndex, subplotIndex, ...
                        varargin)
%                        legendName, scenario, optionsReal, optionsPred, leftOrRight
        % ADDLINE adds settings for a line, which shall be plotted, to the class
        % property lineSeries and updates dependsOnAgents property if necessary
        % Input:
        %   agent       agent handle from which to take variables
        %   variable        name of Variable to take, e.g. 'x'
        %   variableIndex   index of Variable. if [], *all* are taken.
        %   subPlotIndex    index of subplot to plot into
        %   legendName      legend name or cell array of legend names
        %   scenario        Which scenario to take. If [], 1 is used.
        %   optionsReal     cell array with line options for real values
        %   optionsPred     cell array with line options for predicted values
        %   leftOrRight     'left' or 'right' , i.e. which y axes to take
            legendName = {};
            scenario = 1;
            optionsReal = {};
            optionsPred = {};
            leftOrRight = 'left';

            if nargin >= 6
                legendName = varargin{1};
            end
            if nargin >= 7 && ~isempty(varargin{2})
                scenario = varargin{2};
            end
            if nargin >= 8
                optionsReal = varargin{3};
            end
            if nargin >= 9
                optionsPred = varargin{4};
            end
            if nargin >= 10 && ~isempty(varargin{5})
                leftOrRight = varargin{5};
            end

            obj.lineSeries{end+1} = {agent, variable, variableIndex, subplotIndex, ...
                                    legendName, scenario, optionsReal, ...
                                    optionsPred, leftOrRight};

            % Add agent name to dependsOnAgents property if necessary
            if ~any(strcmp(obj.dependsOnAgents, agent.name))
                obj.dependsOnAgents{end+1} = agent.name;
            end

            % Add agent name to subplotDependsOnAgents property if necessary
            if ~any(strcmp(obj.subplotDependsOnAgents{subplotIndex}, agent.name))
                obj.subplotDependsOnAgents{subplotIndex}{end+1} = agent.name;
            end

        end

        function addHeatmap(obj, agent, variable, variableIndexArray, subplotIndex, ...
                             varargin)
%                              scenario, optionsReal, optionsPred
        % addHeatmap adds settings for a heatmap, which shall be plotted, to the class
        % property heatMaps and updates dependsOnAgents property if necessary
        % Input:
        %   agent           agent handle from which to take variables
        %   variable        name of Variable to take, e.g. 'x'
        %   variableIndex   index of Variable. if [], all are taken.
        %   subPlotIndex    index of subplot to plot into
        %   scenario        Which scenario to take. If [], 1 is used.
        %   optionsReal     cell array with line options for real values
        %   optionsPred     cell array with line options for predicted values

            scenario = 1;
            optionsReal = {};
            optionsPred = {};

            if nargin >= 6 && ~isempty(varargin{1})
                scenario = varargin{1};
            end
            if nargin >= 7
                optionsReal = varargin{2};
            end
            if nargin >= 8
                optionsPred = varargin{3};
            end

            % Additionally, save empty cells for possible labeling of the
            % colorbar at Positions 8 and 9
            % and possible colorBarLimits at 10
            obj.heatMaps{end+1} = {agent, variable, variableIndexArray, subplotIndex, ...
                                       scenario, optionsReal, optionsPred, {}, {}, {} };

            % Add agent name to dependsOnAgents property if necessary
            if ~any(strcmp(obj.dependsOnAgents, agent))
                obj.dependsOnAgents{end+1} = agent.name;
            end

            % Add agent name to subplotDependsOnAgents property if necessary
            if ~any(strcmp(obj.subplotDependsOnAgents{subplotIndex}, agent.name))
                obj.subplotDependsOnAgents{subplotIndex}{end+1} = agent.name;
            end


        end

        function setColorBarLabel(obj, subplotIndex, colorBarLabel, varargin)
        % setColorBarLabel sets the lable of the colorbar of the heatmap at subplotIndex

            % Buffer optional options for label
            labelOptions = {};
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    labelOptions{ii} = varargin{1}{ii};
                end
            end

            % First find the corresponding heatmap
            indCorrespondingMap = [];
            for ii = 1 : size(obj.heatMaps, 2)
                if obj.heatMaps{ii}{4} == subplotIndex
                    if isempty(indCorrespondingMap)
                        indCorrespondingMap = ii;
                    else
                        error('PARODIS: TimeSeries: You must not have multiple heatmaps at the same subplot!')
                    end
                end
            end

            % save label arguments as additional information in
            % obj.heatMaps
            if ~isempty(indCorrespondingMap)
                obj.heatMaps{indCorrespondingMap}{8} = colorBarLabel;
                obj.heatMaps{indCorrespondingMap}{9} = labelOptions;
            else
                warning('PARODIS: TimeSeries: Did not find heatmaps in the respective subplot!')
            end
        end

        function setColorBarLimits(obj, subplotIndex, colorBarLimits)
        % SETCOLORMAPLIMITS sets the limits of the colorbar of the colorMap at subplotIndex


            % First find the corresponding colorMap
            indCorrespondingMap = [];
            for ii = 1 : size(obj.heatMaps, 2)
                if obj.heatMaps{ii}{4} == subplotIndex
                    if isempty(indCorrespondingMap)
                        indCorrespondingMap = ii;
                    else
                        error('PARODIS: TimeSeries: You must not have multiple heatmaps at the same subplot!')
                    end
                end
            end

            % save limits as additional information in obj.heatMaps
            if ~isempty(indCorrespondingMap)
                obj.heatMaps{indCorrespondingMap}{10} = colorBarLimits;
            else
                warning('PARODIS: TimeSeries: Did not find heatmaps in the respective subplot!')
            end
        end

        function show(obj, callingAgent, isFinalUpdate)
        % SHOW uses Figure-class implemenation fo show() to set fighandle.Visible to 'on'
        % and then does the actual plotting, i.e. all from lineSeries{...} and
        % heatMaps{...}

            % if Instance does not depend on callingAgent, nothing needs to
            % be done
            if ~any(strcmp(obj.dependsOnAgents, callingAgent.name))
                return
            end
            
            % if individual figure should not be plotted live and this is not the final plot, return 
            if ~obj.plotLive && ~isFinalUpdate 
                return
            end

            % Turn on only for debugging!
%             show@Figure(obj);

            % First clear all axes and reset temporary class properties
            for jj = 1 : size(obj.subplotAxisHandles, 2)
                % Delete only subplots which depend on callingAgent !
                if any(strcmp(obj.subplotDependsOnAgents{jj}, callingAgent.name))
                    cla(obj.subplotAxisHandles{jj});
                    if obj.subplotHasRightY(jj)
                        yyaxis(obj.subplotAxisHandles{jj}, 'right');
                        cla(obj.subplotAxisHandles{jj});
                        yyaxis(obj.subplotAxisHandles{jj}, 'left');
                    end

                    % delete hidden lines from subplot
                    % Delete lines which are hidden
                    for kk = 1 : size(obj.subplotLinesToDelete{jj}, 2)
                        delete(obj.subplotLinesToDelete{jj}{kk});
                    end
                    obj.subplotLinesToDelete{jj} = {};
                end
            end


            obj.subplotAxisPredictionTimesAndWidths = cell(1, obj.nRows*obj.nColumns);

            %% go through lineSeries
            for ii = 1 : size(obj.lineSeries, 2)
                % get arguments for readability
                agent        = obj.lineSeries{ii}{1};
                variableName = obj.lineSeries{ii}{2};
                variableIndex= obj.lineSeries{ii}{3};
                subplotIndex = obj.lineSeries{ii}{4};
                legendName   = obj.lineSeries{ii}{5};
                scenario     = obj.lineSeries{ii}{6};
                optionsReal  = obj.lineSeries{ii}{7};
                optionsPred  = obj.lineSeries{ii}{8};
                leftOrRight  = obj.lineSeries{ii}{9};

                % if line does not belong to a subplot which depends on the
                % callingAgent, then subplot has not been deleted and line
                % does not need to be plotted again
                if ~any(strcmp(obj.subplotDependsOnAgents{subplotIndex}, callingAgent.name))
                    continue
                end

                axesHandle = obj.subplotAxisHandles{subplotIndex};
                if isempty(optionsReal)
                    optionsReal = {'LineStyle', '-'};
                end
                if isempty(optionsPred)
                    optionsPred = {'LineStyle', '--'};
                end
                obj.subplotHasLineSeries(subplotIndex) = 1;

                hold(axesHandle, 'on');
                grid(axesHandle, 'on');

                % only set yyaxis(.., leftOrRight) if necessary. Otherwise
                % it shows values on right side even if leftOrRight ==
                % 'left'
                if strcmp(leftOrRight, 'right')
                    yyaxis(axesHandle, 'right');
                    obj.subplotHasRightY(subplotIndex) = 1;
                end
%                 axis(axesHandle, 'auto');

                % if there are no fixed y-limits, set ylim-mode to 'auto'
                if strcmp(leftOrRight, 'left') && isempty(obj.subplotFixedYLimitsLeft{subplotIndex}) || ...
                      strcmp(leftOrRight, 'right') && isempty(obj.subplotFixedYLimitsRight{subplotIndex})
                   axesHandle.YLimMode = 'auto';
                   axesHandle.YTickMode= 'auto';
                   axesHandle.YTickLabelMode = 'auto';
                end

                hColors = [];

                % Get indices of variable to plot
                if isempty(variableIndex)
                    if any(strcmp({'x', 'u', 'd', 'xVirt', 'uVirt', 'dVirt', 'xDiff', 'uDiff', 'dDiff'}, variableName))
                        variableIndex = 1 : agent.model.(['n_' variableName(1)]);
                    else % evalValues  are always scalar!
                        variableIndex = 1;
                    end
                end

                takeVariableIndex = [];
                % DO THE ACTUAL PLOTTINGS OF LINESERIES
                for jj = 1:length(variableIndex)

                    % if eval or cost function is plotted, than variableIndex
                    % is a string instead of an array -> has to be handled
                    % extra!
                    if ischar(variableIndex)
                        if ~isempty(takeVariableIndex) % second loop -> continue
                            continue
                        end
                        takeVariableIndex = variableIndex;
                    else
                        takeVariableIndex = variableIndex(jj);
                    end

                    % Plot history
                    [plotTimeHistory, plotDataHistory] = obj.readInDataToPlot(agent, variableName, takeVariableIndex, scenario, 'history', 'line');

                    if ~isempty(plotDataHistory) % don't plot at first step / is empty
                        if isempty(legendName) || isempty(legendName{jj}) % if no legend entry, set 'HandleVisibility' to 'off'
                            hReal = stairs(axesHandle, plotTimeHistory, plotDataHistory, '-', 'HandleVisibility', 'off', optionsReal{:});
                            hColors(jj,:) = hReal.Color;
                        else % if there is a legend entry, use 'DisplayName'
                            hReal = stairs(axesHandle, plotTimeHistory, plotDataHistory, '-', 'DisplayName', legendName{jj}, optionsReal{:});
                            hColors(jj,:) = hReal.Color;
                            obj.subplotHasLegend(subplotIndex) = 1;
                            obj.subplotLinesToDelete{subplotIndex}{end+1} = hReal;
                        end
                    end
                    % Plot status ... 
                    [plotTimePred, plotDataPred] = obj.readInDataToPlot(agent, variableName, takeVariableIndex, scenario, 'status', 'line');
                    % ... but only if there is a status to plot
                    if ~isempty(plotDataPred)
                        if isempty(hColors)
                            hPred = stairs(axesHandle, plotTimePred, plotDataPred, '--', 'HandleVisibility', 'off', optionsPred{:});
                        else
                            hPred = stairs(axesHandle, plotTimePred, plotDataPred, '--', 'HandleVisibility', 'off', 'Color', hColors(jj,:), optionsPred{:});
                        end
                        obj.subplotLinesToDelete{subplotIndex}{end+1} = hPred;

                        % save start & end time of prediction horizon to draw grey box later
                        % -> will overwrite previous values, but should not
                        % matter.
                        obj.subplotAxisPredictionTimesAndWidths{subplotIndex} = [agent.status.horizonTime(1), sum(agent.config.T_s)];

                        % Save proper xlimits for plotting
                        newLimits = [agent.history.simulationTime(1), agent.status.horizonTime(end)+agent.config.T_s(end)];
                        obj.updateTempXLimits(subplotIndex, newLimits);
                    else
                        newLimits = [agent.history.simulationTime(1), agent.history.simulationTime(end)];
                        obj.updateTempXLimits(subplotIndex, newLimits);
                    end
                end


            end % end For-loop through lineSeries

            %% Go through heatmaps!
            for ii = 1 : size(obj.heatMaps, 2)
                % get arguments for readability
                agent               = obj.heatMaps{ii}{1};
                variableName        = obj.heatMaps{ii}{2};
                variableIndexArray  = obj.heatMaps{ii}{3};
                subplotIndex        = obj.heatMaps{ii}{4};
                scenario            = obj.heatMaps{ii}{5};
                optionsReal         = obj.heatMaps{ii}{6};
                optionsPred         = obj.heatMaps{ii}{7};
                colorBarLabel       = obj.heatMaps{ii}{8};
                optionsColorBarLabel = obj.heatMaps{ii}{9};
                colorBarLimits      = obj.heatMaps{ii}{10};

                % if heatmap does not belong to a subplot which depends on the
                % callingAgent, then subplot has not been deleted and line
                % does not need to be plotted again
                if ~any(strcmp(obj.subplotDependsOnAgents{subplotIndex}, callingAgent.name))
                    continue
                end


                axesHandle = obj.subplotAxisHandles{subplotIndex};
                hold(axesHandle, 'on');
                obj.subplotHasHeatMap(subplotIndex) = 1;

                % if no variableIndexArray is given, take all indices
                if isempty(variableIndexArray)
                    if any(strcmp({'x', 'u', 'd', 'xVirt', 'uVirt', 'dVirt', 'xDif', 'uDif', 'dDif'}, variableName))
                        variableIndexArray = 1 : agent.model.(['n_' variableName]);
                    else % evalValues are always scalar -> makes no sense for heatmaps?
                        variableIndexArray = 1;
                    end
                end

                % initalize newLimits to zeros
                newLimits = [0, 0];

                % get history and status data to plot
                [timeHistory, dataZHistory] = obj.readInDataToPlot(agent, variableName, variableIndexArray, scenario, 'history', 'colorMap');
                [timeStatus, dataZStatus] = obj.readInDataToPlot(agent, variableName, variableIndexArray, scenario, 'status', 'colorMap');
                valuesY = variableIndexArray;

                if ~isempty(timeHistory) % no history plotting at the beginning
                    [plotXHistory, plotYHistory, plotZHistory] = obj.prepareDataForSurfacePlot(timeHistory, valuesY, dataZHistory, agent.config.T_s(end));
                    haxHistory = surface(obj.subplotAxisHandles{subplotIndex}, plotXHistory, plotYHistory, plotZHistory);
                    % Set Options
                    for ii = 1:2:size(optionsReal, 2)
                        haxHistory.(optionsReal{ii}) = optionsReal{ii+1};
                    end
                    haxHistory.EdgeColor = 'none';

                    newLimits(1) = plotXHistory(1);
                end

                if ~isempty(timeStatus) % no status plotting at the end of the simulation
                    [plotXStatus, plotYStatus, plotZStatus]    = obj.prepareDataForSurfacePlot(timeStatus, valuesY, dataZStatus, agent.config.T_s(end));
                    haxStatus = surface(obj.subplotAxisHandles{subplotIndex}, plotXStatus, plotYStatus, plotZStatus);
                    % Set Options
                    for ii = 1:2:size(optionsPred, 2)
                        haxStatus.(optionsPred{ii}) = optionsPred{ii+1};
                    end

                    newLimits(2) = plotXStatus(end);
                end

                hColorBar = colorbar(axesHandle);
                % set Label of colorBar -> unfortunately, this has to be
                % done every time :(
                if ~isempty(colorBarLabel)
                    ylabel( hColorBar, colorBarLabel );
                    % Set Options
                    for ii = 1:2:size(optionsColorBarLabel, 2)
                        hColorBar.Label.(optionsColorBarLabel{ii}) = optionsColorBarLabel{ii+1};
                    end
                end

                % set limits for colorBar
                if ~isempty(colorBarLimits)
                    caxis(axesHandle, colorBarLimits);
                end

                % adjust y-ticks
                axesHandle.YTick = valuesY+0.5;
                axesHandle.YTickLabels = arrayfun(@(x){num2str(x)}, valuesY);
                axesHandle.YLim = [min(variableIndexArray), max(variableIndexArray)+1];

                % save x-Value for prediction bar
                if ~isempty(timeStatus)
                    obj.subplotAxisPredictionBarAt{subplotIndex} = agent.status.horizonTime(1);
                else
                    obj.subplotAxisPredictionBarAt{subplotIndex} = [];
                end

                % Save proper xlimits for plotting
                obj.updateTempXLimits(subplotIndex, newLimits);
            end % end FOR-loop through heatmaps

            %% Go through every subplot and ...
            for subplotIndex = 1 : size(obj.subplotAxisHandles, 2)

                % if subplot does not depend on the callingAgent, then
                % nothing has to be adjusted
                if ~any(strcmp(obj.subplotDependsOnAgents{subplotIndex}, callingAgent.name))
                    continue
                end

                % ... 1) set axes limits (should start at first point and end at last)
                if obj.subplotHasLineSeries(subplotIndex) || obj.subplotHasHeatMap(subplotIndex)
                    % x-limits
                    obj.setNewXLimits(subplotIndex, isFinalUpdate);
                    % y-limits
                    if ~isempty(obj.subplotFixedYLimitsLeft{subplotIndex})
                        if obj.subplotHasRightY(subplotIndex)
                            yyaxis(obj.subplotAxisHandles{subplotIndex}, 'left');
                        end
                       obj.subplotAxisHandles{subplotIndex}.YLim = obj.subplotFixedYLimitsLeft{subplotIndex};
                    end
                    if ~isempty(obj.subplotFixedYLimitsRight{subplotIndex}) && obj.subplotHasRightY(subplotIndex)
                            yyaxis(obj.subplotAxisHandles{subplotIndex}, 'right');
                            obj.subplotAxisHandles{subplotIndex}.YLim = obj.subplotFixedYLimitsRight{subplotIndex};
                    end
                end

                if obj.doLinkAxes && sum(obj.subplotHasLineSeries==1 | obj.subplotHasHeatMap==1)>=1
                    linkaxes([obj.subplotAxisHandles{obj.subplotHasLineSeries==1 | obj.subplotHasHeatMap==1}], 'x'); % obj.subplotHasLineSeries==1
                else
                    linkaxes([obj.subplotAxisHandles{:}], 'off');
                end
                % ... 2) draw gray box over predictions, if it contains a lineSeries object
                if ~isempty(obj.subplotAxisPredictionTimesAndWidths{subplotIndex})  %isempty(boxesHandles{subPlotIndex})

                    if obj.subplotHasRightY(subplotIndex)
                        yyaxis(obj.subplotAxisHandles{subplotIndex}, 'left');
                    end
                    axis(obj.subplotAxisHandles{subplotIndex}, 'manual')
                    rectangle(obj.subplotAxisHandles{subplotIndex}, 'Position', ...
                        [obj.subplotAxisPredictionTimesAndWidths{subplotIndex}(1) ... % lower left point: x-coordinate
                         obj.subplotAxisHandles{subplotIndex}.YLim(1) ...         % lower left point: y-coordinate
                         obj.subplotAxisPredictionTimesAndWidths{subplotIndex}(2) ... % width
                         obj.subplotAxisHandles{subplotIndex}.YLim(2)-obj.subplotAxisHandles{subplotIndex}.YLim(1)], ... % height
                        'EdgeColor',[0 0 0 0.4], 'FaceColor', [0.7 0.7 0.7 0.1]);
                end

                % ... 2.5) or just a bar if heatmap and etc.
                if ~isempty(obj.subplotAxisPredictionBarAt{subplotIndex})  %isempty(boxesHandles{subPlotIndex})
                    axis(obj.subplotAxisHandles{subplotIndex}, 'manual')
                     plot3(obj.subplotAxisHandles{subplotIndex}, ...
                         [obj.subplotAxisPredictionBarAt{subplotIndex}, obj.subplotAxisPredictionBarAt{subplotIndex}], ...
                         [obj.subplotAxisHandles{subplotIndex}.YLim(1), obj.subplotAxisHandles{subplotIndex}.YLim(2)], ...
                         [0, 0], ...
                         'k-', 'LineWidth', 3.5);
                end
                % ... 3) print legend
                if obj.subplotHasLegend(subplotIndex)
                    legend(obj.subplotAxisHandles{subplotIndex});
                end
            end

            % Show figure at the very end only!
            drawnow;
            show@Figure(obj);
        end % end show()

        function [plotTime, plotData] = readInDataToPlot(obj, agent, variableName, variableIndex, scenario, historyOrStatus, lineOrColorMap)
        %READINDATATOPLOT helper function to get data for plotting in
        %show().
        % Inputs:
        % ...
        % variableIndex:    either single index, array or name (= charArray). If Array, values
        % for all single indices are returned.
        % ....
        % predOrHistory:    either 'history' or 'status'
        % lineOrColor:      either 'line' or 'colorMap' (if colorMap)

        plotTime = [];
        plotData = [];

        % if 'history' at very first time step, return
        if strcmp(historyOrStatus, 'history') && isempty(agent.history.u)
            return
        end
        
        % adjust historyType and variableName if virtual or dif value should be plotted        
        isDif = 0;         
        if endsWith(variableName, 'Virt') % if virtual variable
            
            % if status shall be plotted: return
            if strcmp(historyOrStatus, 'status')
                return
            end
            
            historyType = 'virtualHistory'; 
            variableName = variableName(1:end-4); % delete 'Virt' part for easier handling
            
        elseif endsWith(variableName, 'Diff') % if dif variable
            
            % if status shall be plotted: return
            if strcmp(historyOrStatus, 'status')
                return
            end
            
            isDif = 1; 
            historyType = 'diffHistory'; 
            variableName = variableName(1:end-3); % delete 'Dif' part for easier handling

        else % regular value

            historyType = 'history'; 
            
        end
        
        
        % if status should be plotted ... 
        if strcmp(historyOrStatus, 'status')           
            % ... but there is no prediction available, return (end of
            % simulation)
            if isempty(agent.status.uPred)
                return
            end
            % ... otherwise, adjust variableName
            variableName = [variableName 'Pred'];            
        end

        if isempty(scenario) % use scenario 1 by default
            scenario = 1;
        end

        
        % DEFINE TIME VECTOR
        if strcmp(historyOrStatus, 'history')
            
            if strcmp(lineOrColorMap, 'line')
                if strcmp(variableName, 'x') % % special treatment for x / xVirt / xDif since we know one step more / append last step again
                    plotTime = [agent.history.simulationTime, agent.history.simulationTime(end)+agent.config.T_s(1)];
                else
                    plotTime = agent.history.simulationTime;
                end
                
            elseif strcmp(lineOrColorMap, 'colorMap') % colorMap: take only all for  / xVirt / xDif , otherwise cut last out
                if strcmp(variableName, 'x') % only for x append take all
                    plotTime = agent.history.simulationTime;
                else % for others, cut last out
                    plotTime = agent.history.simulationTime(1:end-1);
                end
            end
        elseif strcmp(historyOrStatus, 'status')
            
            if strcmp(lineOrColorMap, 'line')
                % special treatment for x since we know one step more
                if strcmp(variableName, 'xPred')
                    plotTime = [agent.status.horizonTime(2:end), ...
                        agent.status.horizonTime(end)+agent.config.T_s(1) ...
                        agent.status.horizonTime(end)+sum(agent.config.T_s(1:min(2, end))) ];
                else  % everything else but x
                    plotTime = [agent.status.horizonTime, agent.status.horizonTime(end)+agent.config.T_s(1)];
                end
                
            elseif strcmp(lineOrColorMap, 'colorMap') % colorMap: take only all for x, otherwise cut last out
                if strcmp(variableName, 'xPred') % one step more
                    plotTime = [agent.status.horizonTime(2:end), ...
                        agent.status.horizonTime(end)+agent.config.T_s(1) ];
                else  % everything else but x
                    plotTime = [agent.status.horizonTime];
                end
            end

        end

        % DEFINE DATA VECTOR
        if strcmp(historyOrStatus, 'history')
            if strcmp(lineOrColorMap, 'line') % only for lineSeries append last step again
                % special treatment for x since we know one step more
                if strcmp(variableName, 'x') % append last step again
                    plotData = [agent.(historyType).(variableName)(variableIndex,:), agent.(historyType).(variableName)(variableIndex,end)];
                elseif any(strcmp({'u', 'd'}, variableName)) % input or disturbance
                    plotData = [agent.(historyType).(variableName)(variableIndex,:), agent.(historyType).(variableName)(variableIndex, end)];
                elseif strcmp(variableName, 'eval')  % evaluation functions
                    plotData = [agent.(historyType).evalValues.(variableIndex)(1,:), agent.(historyType).evalValues.(variableIndex)(end)];
                elseif strcmp(variableName, 'cost')  % cost functions
                    plotData = [agent.(historyType).costs.(variableIndex)(1,:), agent.(historyType).costs.(variableIndex)(end)];
                else
                    error('Parodis: TimeSeries: readInDataToPlot(): variableName unknowon')
                end
                
            elseif strcmp(lineOrColorMap, 'colorMap') % for colorMap, no need to take last step twice
                % special treatment for x since we know one step more
                if strcmp(variableName, 'x')
                    plotData = agent.(historyType).(variableName)(variableIndex,:);
                elseif any(strcmp({'u', 'd'}, variableName)) % input or disturbance
                    plotData = agent.(historyType).(variableName)(variableIndex,:);
                elseif strcmp(variableName, 'eval')  % evaluation functions
                    plotData = agent.(historyType).evalValues.(variableIndex)(1,:);
                elseif strcmp(variableName, 'cost')  % cost functions
                    plotData = agent.(historyType).costs.(variableIndex)(1,:);
                else
                    error('Parodis: TimeSeries: readInDataToPlot(): variableName unknowon')
                end
            end

        elseif strcmp(historyOrStatus, 'status')
            
            if strcmp(lineOrColorMap, 'line') % only for lineSeries append last step again
                % special treatment for x since we know one step more
                if strcmp(variableName, 'xPred')
                    plotData = [agent.status.(variableName){scenario}(variableIndex, 2:end), agent.status.(variableName){scenario}(variableIndex, end) ];
                elseif strcmp('uPred', variableName) % input
                    plotData = [agent.status.(variableName)(variableIndex, :), agent.status.(variableName)(variableIndex, end)];
                elseif strcmp('dPred', variableName) % disturbance
                    plotData = [agent.status.(variableName){scenario}(variableIndex, :), agent.status.(variableName){scenario}(variableIndex, end)];
                elseif strcmp(variableName, 'evalPred')  % evaluation functions
                    try % might not yet be set if called from other agent
                        plotData = [agent.status.evalPred.(variableIndex){scenario}(1,:), agent.status.evalPred.(variableIndex){scenario}(end)];
                    catch
                        plotTime = [];
                        plotData = [];
                        return
                    end
                elseif strcmp(variableName, 'costPred')  % cost functions
                    plotData = [agent.status.costsPred.(variableIndex)(1,:), agent.status.costsPred.(variableIndex)(end)];
                else
                    error('Parodis: TimeSeries: readInDataToPlot(): variableName unknowon')
                end
                
            elseif strcmp(lineOrColorMap, 'colorMap') % for colorMap, no need to take last step twice
                % special treatment for x since we know one step more
                if strcmp(variableName, 'xPred')
                    plotData = [agent.status.(variableName){scenario}(variableIndex, 2:end) ];
                elseif strcmp('uPred', variableName) % input
                    plotData = [agent.status.(variableName)(variableIndex, :)];
                elseif strcmp('dPred', variableName) % disturbance
                    plotData = [agent.status.(variableName){scenario}(variableIndex, :)];
                elseif strcmp(variableName, 'evalPred')  % evaluation functions
                    try % might not yet be set if called from other agent
                        plotData = [agent.status.evalPred.(variableIndex){scenario}(1,:)];
                    catch
                        plotTime = [];
                        plotData = [];
                        return
                    end
                elseif strcmp(variableName, 'costPred')  % cost functions
                    plotData = [agent.status.costsPred.(variableIndex)(1,:)];
                else
                    error('Parodis: TimeSeries: readInDataToPlot(): variableName unknowon')
                end
            end
            
        end
        
        % delete temporary difHistory if created before
        if isDif 
            obj.deleteDifHistory(agent, variableName, variableIndex);
        end

        end % end readInDataToPlot()

        function createDifHistory(obj, agent, variableName, variableIndex)
        % helper function to (temporarily) create difHistory for given
        % variable and index 

        if any(strcmp({'u', 'd', 'x'}, variableName))
            agent.difHistory.(variableName) = agent.history.(variableName) - agent.virtualHistory.(variableName);
        elseif strcmp('eval', variableName)
            agent.difHistory.evalValues.(variableIndex) = agent.history.evalValues.(variableIndex) - agent.virtualHistory.evalValues.(variableIndex);
        elseif strcmp('cost', variableName)
            agent.difHistory.costs.(variableIndex) = agent.history.costs.(variableIndex) - agent.virtualHistory.costs.(variableIndex);
        end
        
            
        end % end createDifHistory()
        
        function deleteDifHistory(obj, agent, variableName, variableIndex)
        % helper function to delete (temporary) difHistory of given variable 
        
        if any(strcmp({'u', 'd', 'x'}, variableName))
            agent.difHistory.(variableName) = []; 
        elseif strcmp('eval', variableName)
            agent.difHistory.evalValues = struct; 
        elseif strcmp('cost', variableName)
            agent.difHistory.costs = struct; 
        end
        
            
        end % end deleteDifHistory()
        
        function  updateTempXLimits(obj, subplotIndex, newLimits)
        %UPDATETEMPXLIMITS helper function to update
        %obj.subplotTempXLimits{subplotIndex} with newLimits if necessary
        % Inputs:
        % ...
        % subplotIndex:  Index of subplot
        % newLimist:     Possible new x-limits which might be set

        if ~isempty(obj.subplotFixedXLimits{subplotIndex}) % if there are fixed x limits, no need to update
            return
        end

        if ~( newLimits(2) > newLimits(1) ) % if e.g. no status and thus [0 0], don't set anything
            return
        end

        if obj.doLinkAxes == 0 % no linkd axes
            % -> check if newLimits are bigger than old (if there have been limits before)
            if ~isempty(obj.subplotTempXLimits{subplotIndex})
                if newLimits(1) < obj.subplotTempXLimits{subplotIndex}(1)
                    obj.subplotTempXLimits{subplotIndex}(1)= newLimits(1);
                end
                if newLimits(2) > obj.subplotTempXLimits{subplotIndex}(2)
                    obj.subplotTempXLimits{subplotIndex}(2)= newLimits(2);
                end
            % if there have been none before, take them anyways
            elseif isempty(obj.subplotTempXLimits{subplotIndex})
                obj.subplotTempXLimits{subplotIndex}= newLimits;
            end
        elseif obj.doLinkAxes == 1 % if axes are linked, maximum should always be taken
            hasEntries = ~cellfun(@isempty, obj.subplotTempXLimits);
            if all(~hasEntries) % if all are empty so far
                obj.subplotTempXLimits{subplotIndex}= newLimits;
            else
                maxLimits(1) = min( newLimits(1), min( cellfun(@(x) min(x(1)), obj.subplotTempXLimits(hasEntries)) ) ); %"maximum" lower limit is actually lowest number
                maxLimits(2) = max( newLimits(2), max( cellfun(@(x) max(x(2)), obj.subplotTempXLimits(hasEntries)) ) );
                obj.subplotTempXLimits{subplotIndex}= maxLimits;
            end
        end

        end % end updateTempXLimits()

        function setNewXLimits(obj, subplotIndex, isFinalUpdate)
        %SETNEWXLIMITS helper function to set obj.subplotAxisHandles{subplotIndex}.XLim

        if ~isempty(obj.subplotFixedXLimits{subplotIndex})
            obj.subplotAxisHandles{subplotIndex}.XLim = obj.subplotFixedXLimits{subplotIndex};
        elseif ~isempty(obj.subplotTempXLimits{subplotIndex})
            obj.subplotAxisHandles{subplotIndex}.XLim = obj.subplotTempXLimits{subplotIndex};
        end

        if isFinalUpdate % set final x-limit to Tsim
            obj.subplotAxisHandles{subplotIndex}.XLim = [obj.subplotAxisHandles{subplotIndex}.XLim(1) obj.finalTime];
        end
        end % end setNewXLimits()

        function [xValues, yValues, zValues] = prepareDataForSurfacePlot(obj, x, y, z, addOffsetToX)
        %PREPAREDATAFORSURFACEPLOT appends additional values to x and y on
        %themselves. For z, it appends the last row as well as the last column.
        % Input Arguments:
        %  x                x-Data, must be in horizontal form
        %  y                y-Data, must be in horizontal form
        %  z                z-Data, must be matrix
        %  addOffsetToX     time width which should be added to x at the end
        xValues = [x, x(end)+addOffsetToX];
        yValues = [y, y(end)+1];
        zValuesTemp = [z, z(:,end)]; % first append last column again
        zValues = [zValuesTemp; zValuesTemp(end,:)]; % then append last row again

        end % end prepareDataForSurfacePlot


    end % end methods
end
