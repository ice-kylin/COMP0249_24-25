classdef SLAMSystem < ebe.slam.SLAMSystem

    properties(Access = protected)

        % Although in the lectures we presented three types of states
        % (estimated, predicted, partial) from an implementation point of
        % view it's easier just to store the one set
        x;
        P;
        u;

        % Map stores landmark ID with the indices in the state vector
        landmarkIDStateVectorMap;

        % The linear system used to predict the state and the observation
        systemModel;

        % The map
        map

        % Store of the mean and covariance values
        timeStore;
        xStore;
        PStore;

        % For activity 4
        muckUp;
    end

    methods(Access = public)

        function obj = SLAMSystem(config)

            % Call base class
            obj@ebe.slam.SLAMSystem(config);

            % Set up the discrete time system for prediction
            obj.systemModel = l2.dotbot.SystemModel(config);


            obj.muckUp = false;

            % Set up the event handlers
            obj.registerEventHandler('init', @obj.handleInitializationEvent);
            obj.registerEventHandler('null_obs', @obj.handleNoUpdate);
            obj.registerEventHandler('gps', @obj.handleGPSObservationEvent);
            obj.registerEventHandler('slam', @obj.handleSLAMObservationEvent);
            obj.registerEventHandler('odom', @obj.handleUpdateOdometryEvent);

            % Set the name
            obj.setName('SLAMSystem');
        end

        function success = start(obj)
            start@ebe.slam.SLAMSystem(obj);
            obj.timeStore = [];
            obj.xStore = zeros(l2.dotbot.SystemModel.NP, 0);
            obj.PStore = zeros(l2.dotbot.SystemModel.NP, 0);

            % Set the dictionary which maps landmark ID to coefficient in
            % the state estimate.
            if (isMATLABReleaseOlderThan('R2023b') == true)
                obj.landmarkIDStateVectorMap = dictionary();
                obj.landmarkIDStateVectorMap = insert(obj.landmarkIDStateVectorMap, 0, 0);
            else
                obj.landmarkIDStateVectorMap = configureDictionary("uint32", "double");
            end

            % Get the map data
            if (isfield(obj.config, 'map'))
                obj.map = obj.config.map;
            end

            success = true;
        end

        % Return the current platform estimate
        function [x,P] = platformEstimate(obj)
            x = obj.x(1:l2.dotbot.SystemModel.NP);
            P = obj.P(1:l2.dotbot.SystemModel.NP, 1:l2.dotbot.SystemModel.NP);
        end
        
        function [T, X, PX] = platformEstimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(obj)

            landmarkIds = keys(obj.landmarkIDStateVectorMap);

            numberOfLandmarks = numel(landmarkIds);
           
            x = NaN(l2.dotbot.SystemModel.NL, numberOfLandmarks);
            P = NaN(l2.dotbot.SystemModel.NL, l2.dotbot.SystemModel.NL, numberOfLandmarks);
            
            for l = 1 : numberOfLandmarks
                landmarkId = landmarkIds(l);
                offset = lookup(obj.landmarkIDStateVectorMap, landmarkId);
                idx = offset + [1;2];
                x(:, l) = obj.x(idx);
                P(:, :, l) = obj.P(idx, idx);
            end
            
        end

        function [T, X, PX] = estimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end

        function muckUpCovarianceMatrix(obj, muckUp)
            obj.muckUp = muckUp;
        end
    end

    methods(Access = protected)

        function success = handleNoPrediction(obj)
            obj.x = obj.x;
            obj.P = obj.P;
            success = true;
        end

        function success = handleNoUpdate(obj, ~)
            obj.x = obj.x;
            obj.P = obj.P;
            success = true;
        end

        function success = handlePredictForwards(obj, dT)

            [obj.x, F, Q] = obj.systemModel.predictState(obj.x, obj.u, dT);
            obj.P = F * obj.P * F' + Q;
            success = true;
        end

        function success = handleInitializationEvent(obj, event)
            obj.x = event.data;
            obj.P = event.covariance;
            obj.initialized = true;
            success = true;
        end

        function success = handleGPSObservationEvent(obj, event)
            [zPred, H, R] = obj.systemModel.predictGPSObservation(obj.x);

            nu = event.data - zPred;
            C = obj.P * H';
            S = H * C + R;
            W = C / S;
            obj.x = obj.x + W * nu;
            obj.P = obj.P - W * S * W';
            success = true;
        end

        function success = handleBearingObservationEvent(obj, event)

            x = obj.x;
            P = obj.P;

            % Update each measurement separately
            for s = 1 : numel(event.info)
                sensor = obj.map.sensors.bearing.sensors(event.info(s));
                [zPred, H, R] = obj.systemModel.predictBearingObservation(x, sensor.position, sensor.orientation);

                % Put code down here to complete the implementation of the KF
                nu = event.data(s) - zPred;
                nu = atan2(sin(nu), cos(nu));
                C = P * H';
                S = H * C + R;
                W = C / S;
                x = x + W * nu;
                P = P - W * S * W';
            end
            obj.x = x;
            obj.P = P;
            success = true;
        end

        % Handle a set of measurements of landmarks
        function handleSLAMObservationEvent(obj, event)
            assert(obj.stepNumber == event.eventGeneratorStepNumber)

            % Get the list of landmarks we know about
            knownLandmarkIDs = obj.landmarkIDStateVectorMap.keys();

            % Find the set of known landmarks in the observation set
            [existingLandmarks, idx] = intersect(event.info, knownLandmarkIDs);

            % First update with the known landmarks

            % ACTIVITY 3
            for o = 1 : numel(existingLandmarks)
                % ADD FOR TASK 3

                % Uncomment these lines to find the index of existing
                % landmarks in the state vector
                %offset = lookup(obj.landmarkIDStateVectorMap, existingLandmarks(o));
                %landmarkIdx = offset + [1;2];

                % Finish implementing activity 3 here.
            end

            % The remaining observations are of new landmarks
            [newLandmarks, idx] = setdiff(event.info, existingLandmarks);

            % ACTIVITY 2

            for o = 1 : numel(newLandmarks)

                % Uncomment these lines to store the index of the state
                % vector with the landmark ID
                %offset = length(obj.x);
                %landmarkIdx = offset + [1;2];
                %obj.landmarkIDStateVectorMap = insert(obj.landmarkIDStateVectorMap, newLandmarks(o), offset);

                % Finish implementing activity 2 here
            end

            % ACTIVITY 4
            if (obj.muckUp == true)
                % Finish implementing activity 4 here
            end
        end

        function success = handleUpdateOdometryEvent(obj, event)
                assert(obj.stepNumber == event.eventGeneratorStepNumber);
                obj.u = event.data;
                success = true;
            end

        function storeStepResults(obj)
            % Store the estimate for the future
            obj.timeStore(:, obj.stepNumber + 1) = obj.currentTime;
            obj.xStore(:, obj.stepNumber + 1) = obj.x(1:l2.dotbot.SystemModel.NP);
            obj.PStore(:, obj.stepNumber + 1) = diag(obj.P(1:l2.dotbot.SystemModel.NP, ...
                1:l2.dotbot.SystemModel.NP));
        end
    end
end