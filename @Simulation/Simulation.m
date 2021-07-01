classdef Simulation < handle
    properties
        % simulation time
        T_sim
        
        % simulation name for storing result
        simulationName
        
        % struct of agents
        agents = struct;
        
        % initial seed for seeding random number generator
        initialSeed
        
        % fixed negotation order
        negotiationOrder
        
        % callback for user defined negotiation
        negotiationHandle
        
        % directory in which to store simulation results
        resultDirectory
        
        % data shared between agents
        shared = struct;
        
        % simulation progress data
        progress = struct;
        
        % list of figures to be plotted
        plots = {};
        
        % simulation configuration
        config = struct;
        
        % interactivity app
        interactivity = struct;
        
        % data source manager
        sourceManager;
        
        k_total
    end
    
    properties (Access = protected)
        % largest time step between all agents
        T_s_max = 0
        
        logFileHandle
    end
    
    properties (SetAccess = protected)
        % agent call order within a timestep
        loopOrder
        
        % current global timestep
        k_sim
        
        % whether or not to do logging
        silentMode
        
        k_start
        T
    end
    
    methods
        function obj = Simulation(name, T_sim, addTimestamp)
            % Simulation    Creates a simulation instance
            %   name            Name of the simulation and for the result directory
            %   T_sim           Simulation time in simulation timescale
            %   addTimestamp    (optional) whether to add timestamp to the
            %                   result directory, default false
            
            if nargin < 3
                addTimestamp = false;
            end
            
            obj.T_sim = T_sim;
            obj.simulationName = name;
            obj.silentMode = 0;
            
            obj.sourceManager = SourceManager();
            
            obj.config.enablePlots = true;
            obj.config.livePlot = true;
            obj.config.storeResults = true;
            obj.config.storePlots = true;
            obj.config.autoSave = false;
            obj.config.doCopyCSVSources = false;
            obj.config.doCopyCallingScript = false;
            
            obj.initialSeed = randi(2^31-1);
            
            obj.progress = struct;
            
            obj.progress.loopAgentStep = [];
            obj.progress.loopAgentStepMax = [];
            obj.progress.negotiationStep = [];
            obj.progress.previousProgress = [];
            obj.progress.currentProgress = [];
            obj.progress.printPareto = [];
            obj.progress.printSolverOutput = [];
            obj.progress.previousDebug = [];
            obj.progress.overallInPercent = [];
            
            if addTimestamp
                switch addTimestamp
                    case 'seconds'
                        suffix = datestr(now, '_yyyymmdd_HHMMSS');
                    case 'silent'
                        suffix = '';
                        obj.silentMode = 1;
                    otherwise
                        suffix = datestr(now, '_yyyymmdd');
                end
            else
                suffix = '';
            end
            
            obj.resultDirectory = string([pwd filesep 'Results' filesep char(name) suffix filesep ]);
            
            Simulation.printHeader();
        end
        
        function addAgent(this, agent)
            % addAgent  Adds an agent to the simulation and initialises it
            %   name    Name for the agent
            %   agent   Instance of an Agent class
            
            if ~isequal( class(agent), 'Agent' )
                error('PARODIS Simulation:addAgent agent must be instance of (subclass of) Agent');
            end
            
            agent.simulation = this;
            name = agent.name;
            
            this.log("initialising agent " + name + " ...", true);
            agent.initialise();
            this.log("done.", true);
            
            this.agents.(name) = agent;
            
            this.T_s_max = max(this.T_s_max, agent.config.T_s(1));
            
            % if agent uses the pareto controller
            if agent.doPareto
                if agent.controller.config.interactivity && ~isfield(this.interactivity, 'currentAgent')
                    this.interactivity.currentAgent = agent;
                    switch length(agent.controller.costFunctions)
                        case 2
                            this.interactivity.app = ParetoInteractivity2D(this, this.interactivity.currentAgent, true);
                        case 3
                            pause on
                            this.interactivity.app = ParetoInteractivity3D(this, this.interactivity.currentAgent, true);
                            pause(3) % pause to prevent failure in app initialization
                            pause off
                    end
                    this.interactivity.app.minimizeScreen;
                end
            end
        end
        
        function addAgents(this, varargin)
            % addAgents(agent1, agent2, ...) Wrapper function for adding multiple agents, calls addAgent
            for i=1:length(varargin)
                this.addAgent(varargin{i});
            end
        end
        
        function addPlot(this, figure)
            % addAgent  Adds a figure to the simulation
            %   figure  Instance of an Figure (sub)class
            
            if ~isa(figure, 'Figure')
                error('PARODIS Simulation:addPlot figure must be instance of (subclass of) Figure');
            end
            
            this.plots{end+1} = figure;
            figure.setFinalTime(this.T_sim);
        end
        
        function addPlots(this, varargin)
            % addPlots(figure1, figure2, ...) Wrapper function for adding multiple agents, calls addPlot
            for i=1:length(varargin)
                this.addPlot(varargin{i});
            end
        end
        
        function updatePlots(this, callingAgent, isFinalUpdate)
            % updatePlots(callingAgent)     Updates all configured plots
            % callingAgent     The agent requesting the plot update
            % isFinalUpdate    True if it's the final update before storing plots
            if nargin < 3
                isFinalUpdate = false;
            end
            
            % only actually trigger show, if plotting is enabled
            % and if either live plotting is enabled or it's the actual
            % final show() at the end of the simulation
            if this.config.enablePlots && ( this.config.livePlot || isFinalUpdate )
                for i=1:numel(this.plots)
                    this.plots{i}.show(callingAgent, isFinalUpdate);
                end
            end
        end
        
        function storePlots(this)
            % storePlots()  Stores all configured plots in the result directory
            
            for i=1:numel(this.plots)
                this.plots{i}.print( this.resultDirectory );
            end
        end
        
        function createLoopOrder(this)
            % createLoopOrder   Generates and sets the order in which to call the agents within a global simulation step
            
            if length( fieldnames(this.agents) ) == 1
                this.loopOrder = 1;
                return;
            end
            
            T_s = zeros( 1, length( fieldnames(this.agents) ) );
            
            % get T_s for each agent
            agentNames = fieldnames(this.agents);
            for idx = 1:length(agentNames)
                T_s(idx) = this.agents.(agentNames{idx}).config.T_s(1);
            end
            
            % order agents and T_s so that the slowest is first in the list
            [T_s, orderedAgents] = sort(T_s, 'descend');
            
            order = get_order(T_s, orderedAgents);
            
            % first call: slowest ... fastest
            % fastest executes T(second fastest)/T(fastest) times in
            % between of every execution of second fastest
            % second fastest executes T(third fastest)/(second fastest)
            % and so on => recursive execution order with 1 slowest:
            % [1 repeat([2 repeat([3 .. , T(2)/T(3)]), T(1)/T(2)) ]
            function order = get_order(T_s, agents)
                if length(agents) > 2
                    suborder = get_order(T_s(2:end), agents(2:end));
                else
                    suborder = agents(end);
                end
                
                order = [agents(1) repmat( suborder, 1, T_s(1)/T_s(2)) ];
            end
            
            this.loopOrder = order;
        end
        
        function prepareSimulation(this, k_start)
            if nargin < 2
                k_start = 0;
            end
            
            this.k_start = k_start;
            
            % Get directory from calling script and copy this script to the
            % results directory
            % But only if runSimulation is *not* called from
            % restoreSimulation() / for later time step
            if k_start == 0 && this.config.doCopyCallingScript
                dbOut = dbstack();
                if size(dbOut, 1) >= 2 % does not work if runSimulation() is called by hand
                    scriptName = dbOut(end).file;
                    scriptPath = which(scriptName);
                    copyfile(scriptPath, strcat(this.resultDirectory, scriptName), 'f')
                else
                    warning('Could not copy original calling script!')
                end
            end
            
            this.clearShared();
            
            % clear source manager to avoid problems with CSV sources
            % created on-the-fly at beginning of simulation execution
            this.sourceManager.clear();
            
            % check if T_s are all even multiples
            agentNames = fieldnames(this.agents);
            if length(agentNames) > 1
                combinations = nchoosek(1:length(agentNames), 2);
                for i=1:size( combinations, 1 )
                    T_A = this.agents.(agentNames{ combinations(i, 1) }).config.T_s(1);
                    T_B = this.agents.(agentNames{ combinations(i, 2) }).config.T_s(1);
                    T_min = min(T_A, T_B);
                    T_max = max(T_A, T_B);
                    
                    if mod(T_max, T_min) > 0
                        error('PARODIS simulation:runSimulation agent timesteps are not even multiples');
                    end
                end
            end
            
            this.log("starting simulation", true);
            
            %this.printHeader();
            
            this.T = 0:this.T_s_max:this.T_sim;
            
            % seed random number generator
            rng( this.initialSeed );
            
            this.createLoopOrder();
            
            % create result directory
            if ~exist(this.resultDirectory, 'dir')
                mkdir(this.resultDirectory);
            end
            
            % store seeds (csvwrite kills large integers)
            dlmwrite(this.resultDirectory + "initial_seed.txt", this.initialSeed, 'delimiter', ',', 'precision', '%i');
            
            % set initial progress status
            this.progress.loopAgentStepMax = zeros(1, length(fieldnames(this.agents)));
            for i=1:length(fieldnames(this.agents))
                this.progress.loopAgentStepMax(i) = sum( this.loopOrder == i );
            end
            
            % clear history after k_start, clear all local statuses
            for idx=1:length(agentNames)
                agent = this.agents.(agentNames{idx});
                
                % if timestep seeds have not been set manually, generate
                if isempty(agent.timestepSeeds)
                    seeds = randi(2^31-1, 1, length(0:agent.config.T_s(1):this.T_sim+this.T_s_max)-1);
                    agent.setTimestepSeeds( seeds );
                    agentDir = this.resultDirectory + filesep + agent.name + filesep;
                    dlmwrite( agentDir + "timestep_seeds.csv", seeds', 'delimiter', ',', 'precision', '%i');
                end
                
                k_local = (this.T_s_max / agent.config.T_s(1) * k_start);
                agent.clearHistory(k_local);
                agent.clearStatus(true);
                agent.status.k = k_local;
                
                agent.logConfig();
            end
        end
        
        function iterateSimulation(this, k, externalData)
            if nargin < 3
                externalData = [];
            end
            
            agentNames = fieldnames(this.agents);
            
            this.k_sim = k;
            
            this.progress.loopAgentStep = zeros(1, length(fieldnames(this.agents)));
            this.progress.negotiationStep = 0;
            this.progress.overallInPercent = (k-this.k_start)/(this.k_total-this.k_start)*100;
            
            % if negotiation order is set, follow it
            if ~isempty(this.negotiationOrder)
                this.log( sprintf("k = %i / %i starting negotiation using fixed order", k, this.k_total) );
                
                % call agents in given order
                for name = this.negotiationOrder
                    this.progress.negotiationStep = this.progress.negotiationStep + 1;
                    
                    this.log( ...
                        sprintf( "\t agent = %s, negotiation step %i / %i", ...
                        name{1}, this.progress.negotiationStep, length(this.negotiationOrder) ) ...
                        );
                    
                    if isfield(externalData, name)
                        externalAgentData = externalData.(name);
                    else
                        externalAgentData = [];
                    end
                    
                    this.agents.(name{1}).doNegotiation( externalAgentData );
                end
                % otherwise, if a callback is set, call that
            elseif ~isempty(this.negotiationHandle)
                this.log( sprintf("k = %i / %i starting negotiation using callback", k, this.k_total) );
                if ~isempty(externalData)
                    this.negotiationHandle( this, externalData );
                else
                    this.negotiationHandle( this );
                end
            end
            
            % call agents in loop order
            this.log( sprintf("k = %i / %i simulation step", k, this.k_total) );
            for idx = this.loopOrder
                this.progress.loopAgentStep(idx) = this.progress.loopAgentStep(idx) + 1;
                this.updateProgress();
                this.log( ...
                    sprintf( "\t agent = %s, step %i / %i", ...
                    agentNames{idx}, this.progress.loopAgentStep(idx), this.progress.loopAgentStepMax(idx) ) ...
                    );
                
                agentName = agentNames{idx};
                
                if isfield(externalData, agentName)
                    externalAgentData = externalData.(agentName);
                else
                    externalAgentData = [];
                end
                
                this.agents.(agentName).doStep( externalAgentData );
                
            end
            
            % after an iteration, clear the shared storage
            this.clearShared();
            
            % if auto save is activated
            if this.config.autoSave && this.config.storeResults && k > 0
                % store by given auto save interval, e.g. every 5 steps
                if mod(k, this.config.autoSave) == 0
                    this.log('auto saving');
                    % store all histories, but not plots
                    for idx = 1:length(agentNames)
                        agent = this.agents.(agentNames{idx});
                        agent.log('auto saving');
                        agent.storeHistory();
                    end
                end
            end
        end
        
        function concludeSimulation(this)
            agentNames = fieldnames(this.agents);
            
            % update all plots and store all histories
            for idx = 1:length(agentNames)
                agent = this.agents.(agentNames{idx});
                
                this.updatePlots( agent, true );
                
                if this.config.storeResults
                    agent.storeHistory();
                end
            end
            
            % store updated plots
            if this.config.enablePlots && this.config.storePlots
                this.log("storing plots", true);
                this.storePlots();
            end
            
            % close the opened interactivity app, if it is still running
            if isfield(this.interactivity,'app')
                if this.interactivity.app.isvalid
                    this.interactivity.app.delete;
                end
            end
        end
        
        function runSimulation(this, k_start)
            % runSimulation     Will run the configured simulation
            %
            %   k_start   (optional) global simulation step at which to start running the simulation
            
            if nargin < 2
                k_start = 0;
            end
            
            this.prepareSimulation(k_start);
            
            % simulation loop
            this.k_total = length(this.T)-1;
            for k = k_start:this.k_total
                this.iterateSimulation(k);
            end
            
            this.log("simulation loop done");
            
            % post simulation loop
            this.concludeSimulation();
            
            this.log("done. bye bye!", true);
        end
        
        function restoreSimulation(this, directory, k_start)
            % restoreSimulation(directory, T_start)     Restores a simulation
            %
            % Will load history and RNG seeds from the given directory and then
            % proceed to run the configured simulation with that data
            %
            %   directory   Directory from where to restore data
            %   k_start     (optional) Time step at which to start the simulation. Defaults to last available time step in simulation history
            
            if nargin < 3
                k_start = -1;
            end
            
            this.log("restoring simulation from " + directory, true);
            
            k_start = this.loadDataFromSimulation(directory, k_start);
            
            % copy seeds from restore simulation to new simulation
            agentNames = fieldnames(this.agents);
            for idx = 1:length(agentNames)
                name = agentNames{idx};
                agent = this.agents.(name);
                
                agentDir = this.resultDirectory + name + filesep;
                if ~exist(agentDir + "timestep_seeds.csv", 'file')
                    dlmwrite( agentDir + "timestep_seeds.csv", agent.timestepSeeds', 'delimiter', ',', 'precision', '%i');
                end
            end
            
            this.runSimulation(k_start);
        end
        
        function k_start = loadDataFromSimulation(this, directory, k_start, agentNames)
            directory = string(pwd) + filesep + "Results" + filesep + string(directory);
            if ~exist(directory, 'dir')
                error('PARODIS Simulation: restoreSimulation given directory does not exist');
            end
            this.initialSeed = csvread(directory + filesep + "initial_seed.txt");
            
            % restore history for given agents
            if nargin <= 3
                agentNames = fieldnames(this.agents);
            else
                agentNames = string(agentNames);
            end
            
            % if k_start is set to -1 by restoreSimulation, then it shall be determined automatically to be
            % the last fully available time step within simulation history
            if k_start == -1
                for idx = 1:length(agentNames)
                    name = agentNames{idx};
                    agent = this.agents.(name);
                    
                    agentDir = directory + filesep + name + filesep;
                    
                    if exist(agentDir, 'dir')
                        u_history = csvread(agentDir + "u.csv");
                        k_start = size(u_history, 1) - 1;
                        
                        break;
                    end
                end
            end
            
            % if no agent history exists, i.e. no autosave and simulation failed, set k_start to 0
            % not a very sensible use case for restoring a simulation, but whatever
            if k_start == -1
                k_start = 0;
            end
            
            for idx = 1:length(agentNames)
                name = agentNames{idx};
                agent = this.agents.(name);
                
                % translate global time step k_start into local k
                k = (this.T_s_max / agent.config.T_s(1) * k_start) + 1;
                
                agentDir = directory + filesep + name + filesep;
                
                % if agent can't be recovered, set history to NaN
                if ~exist(agentDir, 'dir')
                    warning("PARODIS Simulation:restoreSimulation data for agent " + name + "could not be found");
                    restoredTime = 0:agent.config.T_s(1):k*agent.config.T_s(1);
                    
                    % for x(end) we need to set given x0 from agent
                    % initialisation
                    % if it violates constraints, bad luck
                    x_history = [restoredTime' NaN(agent.model.n_x, k+1)'];
                    x_history(end, 2:end) = agent.history.x';
                    
                    u_history = [restoredTime(1:end-1)' NaN(agent.model.n_u, k)'];
                    d_history = [restoredTime(1:end-1)' NaN(agent.model.n_d, k)'];
                    
                    % set seeds to empty, so they will be generated automatically
                    timestepSeeds = [];
                    
                    % restore cost function values
                    costNames = fieldnames(agent.controller.costFunctionIndexes);
                    costs_history = struct;
                    for i = 1:length(costNames)
                        costName = costNames{i};
                        
                        costs_history.(costName) = NaN(1, k);
                    end
                    
                    % store each eval fun seperately
                    evalNames = fieldnames(agent.config.evalFuns);
                    eval_history = struct;
                    for i = 1:length(evalNames)
                        evalName = evalNames{i};
                        eval_history.(evalName) = NaN(1, k);
                    end
                    
                    % virtual history is the same in this case
                    x_history_virtual = x_history;
                    u_history_virtual = d_history;
                    d_history_virtual = d_history;
                    costs_history_virtual = costs_history;
                    eval_history_virtual = eval_history;
                    
                % otherwise, recover from actual data
                else
                    x_history = csvread(agentDir + "x.csv");
                    u_history = csvread(agentDir + "u.csv");
                    d_history = csvread(agentDir + "d.csv");
                    
                    is_legacy = false;
                    % legacy check, if simulation was run with or without virtual history
                    if ~exist(agentDir + "virtual", 'dir')
                        is_legacy = true;
                        
                        % in legacy mode, we set virtual history to NaN
                        x_history_virtual = NaN(size(x_history));
                        u_history_virtual = NaN(size(u_history));
                        d_history_virtual = NaN(size(d_history));
                    else
                        x_history_virtual = csvread(agentDir + "virtual" + filesep + "x.csv");
                        u_history_virtual = csvread(agentDir + "virtual" + filesep + "u.csv");
                        d_history_virtual = csvread(agentDir + "virtual" + filesep + "d.csv");
                    end
                    
                    % load seeds
                    timestepSeeds = csvread(agentDir + "timestep_seeds.csv")';
                    
                    % restore cost function values
                    costNames = fieldnames(agent.controller.costFunctionIndexes);
                    costs_history = struct;
                    for i = 1:length(costNames)
                        costName = costNames{i};
                        
                        costData = csvread(agentDir + "costs_" + costName + ".csv");
                        costs_history.(costName) = costData(1:k, 2:end)';
                        
                        if ~is_legacy
                            costData_virtual = csvread(agentDir + "virtual" + filesep + "costs_" + costName + ".csv");
                            costs_history_virtual.(costName) = costData_virtual(1:k, 2:end)';
                        else
                            costs_history_virtual.(costName) = NaN(size( costs_history.(costName) ));
                        end
                    end
                    
                    % store each eval fun seperately
                    evalNames = fieldnames(agent.config.evalFuns);
                    eval_history = struct;
                    for i = 1:length(evalNames)
                        evalName = evalNames{i};
                        evalHistoryFile = agentDir + "eval_" + evalName + ".csv";
                        if exist(evalHistoryFile, 'file')
                            evalData = csvread(evalHistoryFile);
                            eval_history.(evalName) = evalData(1:k, 2:end)';
                            
                            if ~is_legacy
                                evalData_virtual = csvread(agentDir + "virtual" + filesep + "eval_" + evalName + ".csv");
                                eval_history_virtual.(evalName) = evalData_virtual(1:k, 2:end)';
                            else
                                eval_history_virtual.(evalName) = NaN(size(eval_history.(evalName)));
                            end
                            
                        else
                            eval_history.(evalName) = NaN(1, k);
                        end
                    end
                end
                
                agent.history.simulationTime = x_history(1:k+1, 1)';
                agent.history.x = x_history(1:k+1, 2:end)';
                agent.history.u = u_history(1:k, 2:end)';
                agent.history.d = d_history(1:k, 2:end)';
                agent.history.costs = costs_history;
                agent.history.evalValues = eval_history;
                
                agent.virtualHistory.simulationTime = x_history(1:k+1, 1)';
                agent.virtualHistory.x = x_history_virtual(1:k+1, 2:end)';
                agent.virtualHistory.u = u_history_virtual(1:k, 2:end)';
                agent.virtualHistory.d = d_history_virtual(1:k, 2:end)';
                agent.virtualHistory.costs = costs_history_virtual;
                agent.virtualHistory.evalValues = eval_history_virtual;
                
                agent.setTimestepSeeds( timestepSeeds );
                
                % set local time step (counts from zero)
                agent.status.k = k-1;
                
                % if pareto is configured, restore pareto data
                if isequal(agent.controller.type, 'pareto')
                    
                    paretoDir = agentDir + "pareto" + filesep;
                    
                    % if simulation to be restored was run without pareto,
                    % warn and fill history with NaN
                    if ~exist(paretoDir, 'dir')
                        warning(['PARODIS Simulation:restoreSimulation agent ' name ' was run without pareto evaluation']);
                        agent.history.pareto.utopias = NaN(k, length(agent.controller.costFunctions));
                        agent.history.pareto.nadirs = NaN(k, length(agent.controller.costFunctions));
                        agent.history.pareto.fronts = repmat({NaN}, k, 1);
                        agent.history.pareto.paretoParameters = repmat({NaN}, k, 1);
                        agent.history.pareto.frontDeterminationScheme = repmat({NaN}, 1, k);
                    else
                        utopias = csvread(paretoDir + "utopias.csv");
                        nadirs = csvread(paretoDir + "nadirs.csv");
                        
                        agent.history.pareto.utopias = utopias(1:k, 2:end);
                        agent.history.pareto.nadirs = nadirs(1:k, 2:end);
                        
                        data = load(paretoDir + "fronts.mat");
                        agent.history.pareto.fronts = data.fronts;
                        
                        data = load(paretoDir + "paretoParameters.mat");
                        agent.history.pareto.paretoParameters = data.paretoParameters;
                        
                        data = load(paretoDir + "frontDeterminationScheme.mat");
                        agent.history.pareto.frontDeterminationScheme = data.FDS;
                    end
                end
            end
        end
        
        function clearShared(this)
            % clearShared   Clears the shared data between agents
            this.shared = struct;
        end
        
        function log(this, message, display)
            % log(message, display=false)  Will log the given message into the simulation log file
            %   On initial call, it will create missing directories and
            %   clear old log file if present
            
            % Create directory also in silent mode
            if ~exist(this.resultDirectory, 'dir')
                mkdir(this.resultDirectory);
            end
            
            % don't log anything in silentMode
            if this.silentMode
                return
            end
            
            if nargin < 3
                display = false;
            end
            
            if isempty(this.logFileHandle)
                if exist(this.resultDirectory + "log.txt", 'file')
                    tempHandle = fopen(this.resultDirectory + "log.txt", "w");
                    fwrite(tempHandle, '');
                    fclose(tempHandle);
                end
                
                this.logFileHandle = fopen( this.resultDirectory + "log.txt", "a+");
            end
            
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
            logString = sprintf("[%s] %s\n", timestamp, message);
            if display
                fprintf(logString);
            end
            
            fprintf(this.logFileHandle, logString);
        end
        
        function updateProgress(this)
            % create new string and save in this.currentProgress
            this.progress.currentProgress = [];
            this.progress.currentProgress = [this.progress.currentProgress 'Simulation Progress: ' num2str(this.progress.overallInPercent, '%.2f') ' %% \n'];
            
            indentingForAgentMessages = '     ';
            agentNames = fieldnames(this.agents);
            
            if ~isempty(this.progress.loopAgentStepMax) && ~isempty(this.progress.loopAgentStep)
                for idx = 1:length(agentNames)
                    agentMessage = [ 'Agent ' agentNames{idx} ': Step ' num2str(this.progress.loopAgentStep(idx)) ...
                        ' of ' num2str(this.progress.loopAgentStepMax(idx)) ];
                    
                    this.addIndentToCurrentProgress();
                    this.addToCurrentProgress(agentMessage);
                    this.addToCurrentProgress('\n');
                    
                    % add Pareto status if currently in Pareto Optimization
                    if strcmp(this.agents.(agentNames{idx}).controller.type, 'pareto') ...
                            && ~isempty(this.agents.(agentNames{idx}).controller.paretoCurrentStep)
                        
                        paretoMessage = ['Pareto Step: ' num2str(this.agents.(agentNames{idx}).controller.paretoCurrentStep)];
                        
                        this.addIndentToCurrentProgress(2);
                        this.addToCurrentProgress(paretoMessage);
                        
                        % Add max number of steps if applicable (NBI and FPBI so far)
                        if ~isempty(this.agents.(agentNames{idx}).controller.paretoMaxStep)
                            paretoExtraMessage = [' of ' num2str(this.agents.(agentNames{idx}).controller.paretoMaxStep)];
                            this.addToCurrentProgress(paretoExtraMessage);
                        end
                        % Add line break
                        this.addToCurrentProgress('\n');
                    end
                    
                end
                
                % delete old string
                this.eraseDispMessage(this.progress.previousProgress);
            %% print new STring
            fprintf(this.progress.currentProgress);
            %%
            this.progress.previousProgress = this.progress.currentProgress;
            end
        end
        
        function addToCurrentProgress(this, message)
            % Helper function to add a message (char array!) to
            % currentProgress
            this.progress.currentProgress = [ this.progress.currentProgress ...
                message];
        end
        
        function addIndentToCurrentProgress(this, numOfIndents)
            % Helper function to add an indenting to currentProgress
            
            if nargin == 1
                numOfIndents = 1;
            end
            
            indenting = '     ';
            
            for i = 1 : numOfIndents
                this.addToCurrentProgress(indenting);
            end
        end
        
        function eraseDispMessage(this, message)
            % Helper function to delete a message in the command window by
            % printing the appropriate number of '\b'
            if ~isempty(message)
                fprintf(repmat( '\b', 1, size(message, 2) ...
                    -count(message, '\n') ... % count '\n' only as one
                    -count(message, '%%') )); % count '%%' only as one
            end
        end
        
        function debugDisp(this, message)
            % deletes current progess message, prints "message" and then
            % reprints current progress message
            this.eraseDispMessage(this.progress.currentProgress);
            if endsWith(message, '\n')
                fprintf(message);
            else
                if isstring(message)
                    fprintf(strcat(message, "\n"));
                else
                    fprintf([message '\n']);
                end
            end
            fprintf(this.progress.currentProgress);
        end
    end
    
    methods (Static)
        function printHeader()
            version = '1.0.2';
            releaseDate = '2021-05-18';
            message = [
                "-------------------------------------------------------------";
                "PARODIS - Pareto Optimal MPC for Distributed Systems";
                "by Thomas Schmitt, Jens Engel, Matthias Hoffmann";
                sprintf("Release v%s (%s)", version, releaseDate);
                sprintf("Using YALMIP version %s", yalmip('version'));
                "-------------------------------------------------------------";
            ];
            
            % first pad to equal length, than justify to center
            message = strjust(pad(message), 'center');
            
            % join 2:end-1 with "\n |"
            inner = join( message(2:end-1), '|\n|');
            
            % join separator at top and bottom to inner text, don't forget to add 2 dashes each
            message = sprintf("-%s-\n|%s|\n-%s-\n", message(1), inner, message(end));
            
            fprintf(message);
            
        end
    end
    
end
