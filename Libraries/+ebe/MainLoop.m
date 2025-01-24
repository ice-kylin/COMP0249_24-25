classdef MainLoop < ebe.core.ConfigurableComponent

    % This class runs the main loop for the estimator

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
            obj@ebe.core.ConfigurableComponent(config);

            obj.enableGraphics = true;
            obj.graphicsUpdatePeriod = 1;
            obj.estimators = {};
            obj.resultsAccumulators = {};
            obj.viewers = {};
            obj.postDrawActions = {};

        end

        function setEventGenerator(obj, eventGenerator)
            obj.eventGenerator = eventGenerator;
        end

        function addEstimator(obj, estimator)
            obj.estimators{end+1} = estimator;
        end

        function addResultsAccumulator(obj, activity)
            obj.resultsAccumulators{end+1} = activity;
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
            obj.start();

            while (obj.eventGenerator.keepRunning()  == true)
                obj.runOneStep();
            end
        
            obj.stop();

        end

        function runOneStep(obj)

            % Step the event generator
            obj.eventGenerator.step();
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

            % Start the event generator
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