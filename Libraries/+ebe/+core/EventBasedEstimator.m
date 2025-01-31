classdef EventBasedEstimator < ebe.core.ConfigurableComponent
    % EventBasedEstimator: Base class for the estimator
    % 
    % This class implements the base class of an event-based estimation
    % system. Modern robotic systems are inherently multi-sensor fusion
    % systems: they take data from multiple sensors at different times.
    % As a result, there's quite a lot of management of data and timing.
    % To handle this, the estimator is given a sequencer of timestamped
    % events. It then carries out the necessary operations to manage these
    % event.

    properties(Access = public)

        % Set of registered event handlers
        eventHandlers;
        
        % Step number - how many times have we iterated?
        stepNumber;
        
        % The timestamp that the estimate refers to
        currentEstimateTime;

        % The current time
        currentTime;
                
        % Flag to show if debugging is enabled.
        debug;
        
        % Flag to show if the system has been initialized or not
        initialized;
    end

    methods(Access = public)

        % Create the localization system and start it up.
        function obj = EventBasedEstimator(config)

            obj = obj@ebe.core.ConfigurableComponent(config);
            
            % Set empty handlers
            obj.eventHandlers = containers.Map();


        end

        % Process a cell array which is a sequence of events. Each event is
        % processed in the order it appears in the array.
        function processEvents(obj, events)

            % Process all the events
            for e = 1 : length(events)                
                event = events{e};

                % First check it's of the right class.
                assert(isa(event, 'ebe.core.Event'), ...
                    'eventbasedestimator:processevents:wrongobjecttype', ...
                    'The object type is %s', class(event));
                
                % Now do the actual work for this event
                obj.processEvent(event);
            end
        end

        function start(obj)
            % Set the start time to 0.
            obj.currentTime = 0;
            
            % Set the current step number to zero
            obj.stepNumber = 0;
            
            % Set that we need to initialize
            obj.initialized = false;
        end
    end

    methods(Access = protected)
        
        % Process each individual event. It is assumed these have been
        % sorted in chronological order.

        function success = processEvent(obj, event)

            % Flag determines if we should store a result; we should not
            % increment the step number if the estimator is not initialized
            incrementStepNumber = obj.initialized;

            % First see if we have to do a prediction step. The estimator
            % can only predict if it's been initialized.

            % Figure out the delta from the current estimate time
            dT = event.time() - obj.currentTime;

            % We only predict if the estimator is initialized
            if (obj.initialized == true)
    
                % Time should be non-negative otherwise there's a timestamp
                % skew.
                assert(dT >= 0);

                % If dT is less than minDT, no prediction is needed
                % otherwise, predict forwards. Each prediction step has a
                % maximum length of maxDT.
                if (dT <= obj.config.minDT)
                    obj.handleNoPrediction();
                    incrementStepNumber = false;
                else
                    predictTime = obj.currentTime;
                    while (dT > 0)
                        dTStep = min(dT, obj.config.maxDT);
                        predictTime = predictTime + dTStep;
                        obj.handlePredictForwards(dT);
                        dT = dT - dTStep;
                    end
                end
                obj.currentTime = event.time();
            end

            % 
            if (incrementStepNumber == true)
                % Increment the step number
                obj.stepNumber = obj.stepNumber + 1;
            end

            % Dispatch an event to the corresponding handler
            if obj.eventHandlers.isKey(event.type())
                eventHandler = obj.eventHandlers(event.type());
                eventHandler(event);
            else
                warning('No handler registered for event type: %s', event.type());
                obj.registerEventHandler(event.type(), @obj.ignoreUnknownEventType);
            end

            % Store a step results. Note this catches the cases that either
            % (a) the estimator was just initialized or (b) the prediction
            % interval is sufficiently long that we've bumped to a new
            % time step.
            if (obj.initialized == true) && (incrementStepNumber == true)
                % Handle any post-event activities
                obj.storeStepResults();
            end

            %fprintf('Estimator: %d %f\n',obj.stepNumber, obj.currentTime)

            success = true;

        end

        % TODO: VALIDATE
        function registerEventHandler(obj, eventType, eventHandler)
            obj.eventHandlers(eventType) = eventHandler;
        end

        function success = ignoreUnknownEventType(~, ~)
            success = true;
        end

    end

    methods(Access = protected, Abstract)

        % This method handles the case that no prediction happens
        handleNoPrediction(obj);

        % This handles the case that we predict to a given time
        handlePredictForwards(obj, dT);

        storeStepResults(obj);

    end
end