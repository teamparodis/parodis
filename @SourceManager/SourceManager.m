classdef SourceManager < handle
    %SOURCEMANAGER Handles accessing data sources for parameters and disturbances
    % Caches data coming from CSV sources to avoid unnecessary IO and parsing 
    
    properties(Access=private)
        sources;
        csvData;
    end
    
    methods
        function obj = SourceManager()
            obj.sources = {};
            obj.csvData = {};
        end
    end
    
    methods(Static, Access=private)
        function data = getDataFromCSV(T, csvData, callingAgent, numScenarios)
            % load raw data from csv file
            % first column timestamp in sim time
            
            % build time vector from k, sim.T_s, agent.Ts(k)
            % interp1 at time vector
            
            % first column in CSV is time, following columns are data entries
            try
                time = csvData(:, 1)';
                entries = csvData(:, 2:end);
                
                N_pred = length(callingAgent.config.T_s);
                n_d = callingAgent.model.n_d;
                % Check whether a 'proper' prediction-source is used, i.e. with N_pred*n_d
                % predictions for every time step, or file format as for real data,
                % i.e. only n_d values at every time step! // or other for
                % parameters...
                if size(entries, 2) == n_d*N_pred % pred data format
                    predData = zeros(n_d, N_pred);  %Preallocation for predData
                    %Search for the time index corresponding to current step
                    [~, index] = min(abs(time - T(1)));
                    for jj = 1 : n_d
                        predData(jj,:) = entries(index, 1+(jj-1)*N_pred : jj*N_pred);  %Read predictions for every disturbace
                    end
                    data = predData;
                else % hard to distinguish between real disturbance source or parameter source
                    % -> actually, we'd need another input argument with
                    % parameter dimension (if parameter is wanted)
                    %%% if size(entries, 2) == n_d % real data format
                    data = interp1(time, entries, T)';
                    % interp1 will always return column vectors if input is a
                    % vector not a matrix, so if it's a vector we need to transpose
                    % the retrieved data
                    if size(entries, 2) == 1
                        data = data';
                    end
                end
                
                data = repmat( {data}, numScenarios, 1);
                
            catch exception
                error("PARODIS SourceManager:getDataFromCSV: could not read from source\n" + getReport(exception));
            end
        end
    end
    
    methods
        function clear(this)
            this.sources = {};
            this.csvData = {};
        end
        
        function [data] = getDataFromSource(this, T, source, callingAgent, agents, numScenarios)
            % getDataFromSource     Retrieves from a given source, where the source can
            %                       be a function handle, a path to a CSV file, or a constant matrix
            % Input:
            %   T               time vector at which to retrieve data
            %   source          function handle, path to CSV or constant matrix
            %   callingAgent    instance of class Agent
            %   agents          struct with all currently available Agents
            %   numScenarios    number of scenarios to return
            % Output:
            %   data            if source is not constant, cell array with numScenarios cells, each cell is matrix with length(T) columns
            
            % is source a filename?
            if isstring(source) || ischar(source)
                
                % convert source to char to avoid any string vs char array issues
                source = char(source);
                
                % check if file is already in cache
                index = find(strcmp(this.sources, source), 1);
                
                % if not
                if isempty(index)
                    % load from CSV
                    try
                        csvData = csvread(source);
                        if callingAgent.simulation.config.doCopyCSVSources
                            % copy CSV-file to results directory
                            if ~exist(strcat(callingAgent.simulation.resultDirectory, '/sources/'), 'dir')
                                mkdir(strcat(callingAgent.simulation.resultDirectory, '/sources/'));
                            end
                            [~, fName, fExt] = fileparts(source);
                            copyfile(source, strcat(callingAgent.simulation.resultDirectory, '/sources/', fName, fExt));
                        end
                    catch exception
                        error("PARODIS SourceManager:getDataFromSource: could not read from source\n" + getReport(exception));
                    end
                    
                    % and then store in cache
                    this.sources{end+1} = source;
                    this.csvData{end+1} = csvData;
                    
                    % otherwise retrieve from cache
                else
                    csvData = this.csvData{index};
                end
                
                data = this.getDataFromCSV(T, csvData, callingAgent, numScenarios);
                
            elseif isa(source, 'function_handle')
                % call source with
                data = source(T, callingAgent, agents, numScenarios);
                % if source is a static matrix, return
            elseif isnumeric(source) && ismatrix(source)
                data = repmat({source}, numScenarios, 1);
            else
                error("PARODIS SourceManager:getDataFromSource: invalid source given");
                % TODO: fehler werfen?
            end
        end
    end
    
end
