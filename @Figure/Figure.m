classdef Figure < handle
    % Superclass for TimeSeries, ParetoPlot etc.
    
    properties
        figHandle = [];
        subplotAxisHandles = {}; 
        figureOptions = {};
        subplotOptions = {};
        nRows
        nColumns
        saveAsTypes = {};
        plotLive = 1;
        name
        finalTime
    end
    methods
        % Constructor
        function obj = Figure(name, nRows, nColumns, varargin)
        %FIGURE constructor for Figure-class. 
        % Input: 
        %   name        figure name, e.g. for saved plot
        %   nRows       number of rows for subplots 
        %   nColumns    number of columns for subplots
        %   saveAsTypes [optional] types to save plot as. Default: {'eps',
        %   'png', 'fig'}
            obj.name = name;
            obj.nRows = nRows;
            obj.nColumns = nColumns;
            if numel(varargin) == 1 && ~isempty(varargin{1})
                obj.saveAsTypes = varargin{1};
            else
                obj.saveAsTypes = {'eps', 'fig', 'png'};
            end
            obj.figHandle = figure('visible','off');
            indAxes = 1;
            for ii = 1 : nRows
                for jj = 1 : nColumns
                    obj.subplotAxisHandles{indAxes} = subplot(nRows, nColumns, indAxes); 
                    indAxes = indAxes + 1; 
                end
            end
        end
        

        function setFigureOptions(obj, options)
        %SETFIGUREOPTIONS updates options in figure handle
        % Input: 
        %   options     Cell array with names and values of options such that
        %   figHandle.(options{i}) = options{i+1} is valid 
            obj.figureOptions = options;
            for ii = 1:2:size(options, 2)
                obj.figHandle.(options{ii}) = options{ii+1};
            end
        end
        
        function setAxisOptions(obj, subplotIndex, options)
        % SETAXISOPTIONS sets options for given axes / subplot. 
        % WARNING: can't be used for 'XLabel', 'YLabel' etc.
        % Input: 
        %   subplotIndex:   scalar index of subplot
        %   options:        Cell array with names and values of options
        %                   such that axes.(options{i}) = options{i+1} is
        %                   valid
            if subplotIndex > obj.nRows*obj.nColumns
                warning('PARODIS: Figure: setAxisOptions: Given subplotIndex higher than total number of subplots!')
            end
            obj.subplotOptions{subplotIndex} = options;
            for ii = 1:2:size(options, 2)
                obj.subplotAxisHandles{subplotIndex}.(options{ii}) = options{ii+1};
            end
        end
        
        function setTitle(obj, myTitle, varargin)
        % SETTITLE tries to use sgtitle() to set a proper title for 
        % complete figure. If old Matlab version is used and >1 subplots,
        % then it only updates the window's name. 
        % Input: 
        %   myTitle         title as string
        %   titleOptions    [optional] Cell array with names and values of
        %                   options such that sgt.(options{i}) = options{i+1} 
        %                   is valid
        
            % Buffer optional options for title
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    titleOptions{ii} = varargin{1}{ii};
                end
            else
                titleOptions = {};
            end
        
            if exist('sgtitle') == 0 % old Matlab version without sgtitle()-Function
                if obj.nRows*obj.nColumns == 1 % only 1 subplot
                    title(obj.subplotAxisHandles{1}, myTitle);
                    % Set Options
                    for ii = 1:2:size(titleOptions, 2)
                        obj.subplotAxisHandles{1}.Title.(titleOptions{ii}) = titleOptions{ii+1};
                    end
                else
                    % Multiple Subplots -> unclear on which to put title,
                    % thus use only as figure name. 
                    obj.figHandle.NumberTitle = 'off'
                    obj.figHandle.Name = myTitle; 
                end
            else % new Matlab version, we can use sgtitle()
                sgt = sgtitle(myTitle);
                for ii = 1:2:size(titleOptions, 2)
                    sgt.(titleOptions{ii}) = titleOptions{ii+1};
                end
            end

        end
        
        
        function setSubplotTitle(obj, subplotIndex, myTitle, varargin)
        % SETSUBPLOTTITLE sets title for specific subplot
        % Input: 
        %   subplotIndex    Sclar index of subplot
        %   myTitle         title as string
        %   titleOptions    [optional] Cell array with names and values of
        %                   options such that axes.Title.(options{i}) = options{i+1} 
        %                   is valid, e.g. {'Color', 'red', 'FontSize', 10}
            
            % Buffer optional options for title
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    titleOptions{ii} = varargin{1}{ii};
                end
            else
                titleOptions = {};
            end
            
            if subplotIndex > obj.nRows*obj.nColumns
                warning('PARODIS: Figure: setSubplotTitle: Given subplotIndex higher than total number of subplots!')
            end
            
            title(obj.subplotAxisHandles{subplotIndex}, myTitle);
            % Set Options
            for ii = 1:2:size(titleOptions, 2)
                obj.subplotAxisHandles{subplotIndex}.Title.(titleOptions{ii}) = titleOptions{ii+1};
            end
            
        end
        
        function setXLabel(obj, subplotIndex, myXLabel, varargin)
        % SETXLABEL sets xLabel for specific subplot
        % Input: 
        %   subplotIndex    Sclar index of subplot. if empty, all are
        %   taken
        %   myXLabel        xlabel as string
        %   labelOptions    [optional] Cell array with names and values of
        %                   options such that axes.XLabel.(options{i}) = options{i+1} 
        %                   is valid, e.g. {'Color', 'red', 'FontSize', 10}
            if subplotIndex > obj.nRows*obj.nColumns
                warning('PARODIS: Figure: setXLabel: Given subplotIndex higher than total number of subplots!')
            end
            
            % Buffer optional options for title
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    labelOptions{ii} = varargin{1}{ii};
                end
            else
                labelOptions = {};
            end
            
            if isempty(subplotIndex)
                subplotIndex = 1 : numel(obj.subplotAxisHandles); 
            end
            
            for kk = 1 : length(subplotIndex)
                xlabel(obj.subplotAxisHandles{subplotIndex(kk)}, myXLabel);
                % Set Options
                for ii = 1:2:size(labelOptions, 2)
                    obj.subplotAxisHandles{subplotIndex(kk)}.XLabel.(labelOptions{ii}) = labelOptions{ii+1};
                end
            end
            

            
        end
        
        function setYLabel(obj, subplotIndex, myYLabel, leftOrRight, varargin)
        % SETYLABEL sets xLabel for specific subplot
        % Input: 
        %   subplotIndex    Sclar index of subplot
        %   myYLabel        ylabel as string
        %   labelOptions    [optional] Cell array with names and values of
        %                   options such that axes.YLabel.(options{i}) = options{i+1} 
        %                   is valid, e.g. {'Color', 'red', 'FontSize', 10}
            if subplotIndex > obj.nRows*obj.nColumns
                warning('PARODIS: Figure: setYLabel: Given subplotIndex higher than total number of subplots!')
            end
            
            if isempty(subplotIndex)
                subplotIndex = 1;
            end
            % Buffer optional options for y-label
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    labelOptions{ii} = varargin{1}{ii};
                end
            else
                labelOptions = {};
            end
            
            if nargin == 3 % no leftOrRight given, but "obj" counts, too
                leftOrRight = 'left'; 
            end
            
            if strcmp(leftOrRight, 'right')
                yyaxis(obj.subplotAxisHandles{subplotIndex}, 'right')
            end
            ylabel(obj.subplotAxisHandles{subplotIndex}, myYLabel);
            if strcmp(leftOrRight, 'right') % set back to left
                yyaxis(obj.subplotAxisHandles{subplotIndex}, 'left')
            end
            % Set Options
            for ii = 1:2:size(labelOptions, 2)
                obj.subplotAxisHandles{subplotIndex}.YLabel.(labelOptions{ii}) = labelOptions{ii+1};
            end
            
        end
        
        function setZLabel(obj, subplotIndex, myZLabel, varargin)
        % SETZLABEL sets xLabel for specific subplot
        % Input: 
        %   subplotIndex    Sclar index of subplot
        %   myZLabel        zlabel as string
        %   labelOptions    [optional] Cell array with names and values of
        %                   options such that axes.ZLabel.(options{i}) = options{i+1} 
        %                   is valid, e.g. {'Color', 'red', 'FontSize', 10}
            if subplotIndex > obj.nRows*obj.nColumns
                warning('PARODIS: Figure: setZLabel: Given subplotIndex higher than total number of subplots!')
            end
            
            if isempty(subplotIndex)
                subplotIndex = 1;
            end
            % Buffer optional options for title
            if ~isempty(varargin)
                for ii = 1 : size(varargin{1},2)
                    labelOptions{ii} = varargin{1}{ii};
                end
            else
                labelOptions = {};
            end
            
            ylabel(obj.subplotAxisHandles{subplotIndex}, myZLabel);
            % Set Options
            for ii = 1:2:size(labelOptions, 2)
                obj.subplotAxisHandles{subplotIndex}.ZLabel.(labelOptions{ii}) = labelOptions{ii+1};
            end
            
        end    
        
        function show(obj)
        % SHOW sets fighandle.Visible to 'on'
            obj.figHandle.Visible = 'on'; 
            
        end  
        
        function print(obj, directory)
        % PRINT saves figure in directory for types as defined in obj.saveAsTypes
        % Input: 
        %   directory   Absolute path to directory where saves shall be
        %               saved.
            warning('off', 'MATLAB:MKDIR:DirectoryExists');
            if( isempty(directory) )
                warning('PARODIS: Figure: print: No Directory given. Plots can''t be saved!')
                return
            else
                plotPath = directory + filesep + "plots" + filesep; 
            end
            
            mkdir(plotPath);
            
            for ii=1:numel(obj.saveAsTypes)
                type = char( obj.saveAsTypes{ii} );
                % if file extension is eps, type for saveas has to be epsc
                % to preserve colour, otherwise it's black and white only
                
                if isequal(type, 'eps')
                    type = 'epsc';
                end
                
                saveas(obj.figHandle, plotPath + obj.name, type);
            end
            
            warning('on', 'MATLAB:MKDIR:DirectoryExists');
            
        end
        
        function setFinalTime(obj, Tsim)
            obj.finalTime = Tsim;
        end
        
    end
end