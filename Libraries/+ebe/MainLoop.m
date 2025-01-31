classdef MainLoop < ebe.core.ConfigurableComponent
    % MainLoop: A class which manages the entire loop for an ebe session,
    % from starting up estimators and graphics, to running the main loop
    % with the event generator, to shutting the system back down again.

    properties(Access = protected)

        % Flag shows if the main loop is running
        isRunning;

        % The event generator
        eventGenerator;

        % The set of estimators
        estimators;

        % The set of post step activities
        resultsAccumulators;

        % The set of viewers
        viewers;

        % The set of post draw activities
        postDrawActions;

        % Flag to show if graphics are enabled
        enableGraphics;

        % Flag to show the regularity with which the graphics should be
        % updated
        graphicsUpdatePeriod;

    end

    methods(Access = public)

        function obj = MainLoop(config)
            % MainLoop Constructor for MainLoop
            %
            % Syntax:
            %   mainLoop = MainLoop(config);
            %
            % Description:
            %   Constructor for an instance of a MainLoop object.
            %
            % Inputs:
            %   config - (struct)
            %       The configuration structure
            %
            % Outputs:
            %   obj - (handle)
            %      An instance of a MainLoop

            % Call the base class constructor
            obj@ebe.core.ConfigurableComponent(config);

            % Clear out all the entries and set defaults
            obj.enableGraphics = true;
            obj.isRunning = false;
            obj.graphicsUpdatePeriod = 1;
            obj.estimators = {};
            obj.resultsAccumulators = {};
            obj.viewers = {};
            obj.postDrawActions = {};

        end

        function setEventGenerator(obj, eventGenerator)
            % SETEVENTGENERATOR set the event generator for the mainloop
            %
            % Syntax:
            %   mainLoop.setEventGenerator(eventGenerator)
            %
            % Description:
            %   The main loop must have a single event generator which is
            %   responsible for generating events. This method registers
            %   the generator. Note the generator can only be set while the
            %   main loop is not running
            %
            % Inputs:
            %   eventGenerator - (ebe.core.EventGenerator)
            %       The event generator object

            % First check the object is of the right type
            assert(isa(eventGenerator, 'ebe.core.EventGenerator'), ...
                'ebe:mainloop:invalideventgeneratortype', ...
                'The event generator should inherit from ebe.core.EventGenerator');

            % Second check we aren't running
            assert(obj.isRunning == false, 'ebe:mainloop:setwhilerunning', ...
                'The method was called while the mainloop is running')
            obj.eventGenerator = eventGenerator;
        end

        function addEstimator(obj, estimator)
            % ADDESTIMATOR add an estimator to the mainloop
            %
            % Syntax:
            %   mainLoop.addEstimator(estimator)
            %
            % Description:
            %   Add the estimator to the list of estimators which will be
            %   fired when events are available. No order is specified in
            %   which esimators are fired.
            %
            % Inputs:
            %   eventGenerator - (ebe.core.EventBasedEstimator)
            %       The estimator
            %
            % See Also:
            %   ebe.core.EventBasedEstimator

            obj.estimators{end+1} = estimator;
        end

        function addResultsAccumulator(obj, resultsAccumulator)
            % ADDRESULTSACCUMULATOR add a results accumulator to the mainloop
            %
            % Syntax:
            %   mainLoop.addResultsAccumulator(resultsAccumulator);
            %
            % Description:
            %   Add a results accumulator to the mainloop. Results
            %   accumulators collect data from the event generator and the
            %   estimator to do things like compute errors.
            %
            % Inputs:
            %   eventGenerator - (ebe.core.ResultsAccumulator)
            %       The estimator
            %
            % See Also:
            %   ebe.core.ResultsAccumulator
           
            obj.resultsAccumulators{end+1} = resultsAccumulator;
        end

        function addViewer(obj, viewer)
            obj.viewers{end+1} = viewer;
        end

        function addPostDrawAction(obj, activity)
            obj.postDrawActions{end+1} = activity;
        end        

        function setGraphicsUpdatePeriod(obj, graphicsUpdatePeriod)
            obj.graphicsUpdatePeriod = graphicsUpdatePeriod;
        end

        function run(obj)
            % RUN execute the mainloop
            %
            % Syntax:
            %   mainLoop.run();
            %
            % Description:
            %   Run the mainloop from start to finish. After starting all
            %   the components, the runOneStep method is called repeatedly
            %   until the event generator says its done. At that point, all
            %   the registered components are stopped.
            %
            % See Also:
            %   runOneStep, start, stop
            
            % Start all the components
            obj.start();

            % Keep executing the inner one step until the event generator
            % says its done
            while (obj.eventGenerator.keepRunning()  == true)
                obj.runOneStep();
            end
        
            % Stop all the components
            obj.stop();

        end

        function runOneStep(obj)
            % RUNONESTEP run a single step in the mainloop
            %
            % Syntax:
            %   mainLoop.runOneStep();
            %
            % Description:
            %   Run a single step in the mainloop: the event generator
            %   yields a set of events which are dispatched to the
            %   estimators. The results are collected and the graphics
            %   updated. Note that if this method is used, start and stop
            %   will have to be called externally.
            %
            % See Also:
            %   run, start, stop

            % Step the event generator
            obj.eventGenerator.step();

            % Get the events
            events = obj.eventGenerator.events();

            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.processEvents(events);
            end

            % Handle if there are any post step activities
            for p = 1 : numel(obj.resultsAccumulators)
                obj.resultsAccumulators{p}.collectResults();
            end

            % Finish here if graphics is disabled
            if (obj.enableGraphics == false)
                return
            end

            % Do not update if we haven't reached the next graphics update
            % step
            if (rem(obj.eventGenerator.stepCount(), obj.graphicsUpdatePeriod) ~= 0)
                return
            end

            % Run the visualizers
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.visualize(events);
            end
            drawnow

            % Handle if there are any post visualize activities
            for p = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.run();
            end
        end

        function start(obj)
            % START start all the registered components
            %
            % Syntax:
            %   mainLoop.start();
            %
            % Description:
            %   The start method is called on the event generator, the
            %   estimators, results accumulators, graphics viewers and post
            %   draw actions. This is done automatically by the run method,
            %   or must be called manually if the runOneStep method is
            %   used.
            %
            % See Also:
            %   run, runOneStep, stop            
            
            % Flag that the component is running, so you can't change
            % anything
            obj.isRunning = true;

            % Start the event generator
            obj.eventGenerator.start();
            
            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.start();
            end

            % Handle if there are any post step activities
            for p = 1 : numel(obj.resultsAccumulators)
                obj.resultsAccumulators{p}.setEventGeneratorAndEstimators(obj.eventGenerator, obj.estimators);
                obj.resultsAccumulators{p}.start();
            end

            % Handle if there are any viewers
            if (obj.enableGraphics == false)
                return
            end
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.start();
            end

            % Handle if there are any post visualize activities
            for p = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.start();
            end
        end

        function stop(obj)
            % STOP stop all the registered components
            %
            % Syntax:
            %   mainLoop.stop();
            %
            % Description:
            %   The stop method is called on the event generator, the
            %   estimators, results accumulators, graphics viewers and post
            %   draw actions. This is done automatically by the run method,
            %   or must be called manually if the runOneStep method is
            %   used.
            %
            % See Also:
            %   run, runOneStep, start
            
            % Stop the event generator
            obj.eventGenerator.stop();
            
            % Handle if there are any estimators
            for e = 1 : numel(obj.estimators)
                obj.estimators{e}.stop();
            end

            % Handle if there are any post step activities
            for p = 1 : numel(obj.resultsAccumulators)
                obj.resultsAccumulators{p}.stop();
            end

            % Handle if there are any viewers
            if (obj.enableGraphics == false)
                return
            end
            for v = 1 : numel(obj.viewers)
                obj.viewers{v}.stop();
            end

            % Handle if there are any post visualize activities
            for v = 1 : numel(obj.postDrawActions)
                obj.postDrawActions{p}.stop();
            end

            % Show that this is no longer running
            obj.isRunning = false;

        end
    end
end