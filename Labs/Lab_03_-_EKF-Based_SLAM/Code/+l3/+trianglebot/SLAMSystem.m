classdef SLAMSystem < ebe.slam.SLAMSystem

    properties(Access = protected)

        % Although in the lectures we presented three types of states
        % (estimated, predicted, partial) from an implementation point of
        % view it's easier just to store the one set
        x;
        P;
        u;
        sigmaU;

        % Map stores landmark ID with the indices in the state vector
        landmarkIDStateVectorMap;

        % The linear system used to predict the state and the observation
        systemModel;

        % The map
        scenario;

        % Store of the mean and covariance values
        timeStore;
        xStore;
        PStore;

        % For activity 3
        updateKnownLandmarks;
    end

    methods(Access = public)

        function obj = SLAMSystem(config)

            % Call base class
            obj@ebe.slam.SLAMSystem(config);

            % Set up the discrete time system for prediction
            obj.systemModel = l3.trianglebot.SystemModel(config);

            % Update landmarks by default
            obj.updateKnownLandmarks = true;

            % Set up the event handlers
            obj.registerEventHandler('init', @obj.handleInitializationEvent);
            obj.registerEventHandler('null_obs', @obj.handleNoUpdate);
            obj.registerEventHandler('gps', @obj.handleGPSObservationEvent);
            obj.registerEventHandler('slam', @obj.handleSLAMObservationEvent);
            obj.registerEventHandler('odom', @obj.handleUpdateOdometryEvent);
            obj.registerEventHandler('compass', @obj.handleCompassObservationEvent);

            % Set the name
            obj.setName('SLAMSystem');
        end

        function setUpdateKnownLandmarks(obj, updateKnownLandmarks)
            obj.updateKnownLandmarks = updateKnownLandmarks;
        end

        function success = start(obj)
            start@ebe.slam.SLAMSystem(obj);

            % Set up initial store of results
            obj.timeStore = [];
            obj.xStore = zeros(l3.trianglebot.SystemModel.NP, 0);
            obj.PStore = zeros(l3.trianglebot.SystemModel.NP, 0);

            % Set the dictionary which maps landmark ID to coefficient in
            % the state estimate.
            obj.landmarkIDStateVectorMap = configureDictionary("uint32", "double");

            % Get the map data
            if (isfield(obj.config, 'scenario'))
                obj.scenario = obj.config.scenario;
            end

            success = true;
        end

        % Return the current platform estimate
        function [x,P] = platformEstimate(obj)
            x = obj.x(1:l3.trianglebot.SystemModel.NP);
            P = obj.P(1:l3.trianglebot.SystemModel.NP, 1:l3.trianglebot.SystemModel.NP);
        end
        
        function [T, X, PX] = platformEstimateHistory(obj)
            T = obj.timeStore;
            X = obj.xStore;
            PX = obj.PStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(obj)

            landmarkIds = keys(obj.landmarkIDStateVectorMap);

            numberOfLandmarks = numel(landmarkIds);
           
            x = NaN(l3.trianglebot.SystemModel.NL, numberOfLandmarks);
            P = NaN(l3.trianglebot.SystemModel.NL, l3.trianglebot.SystemModel.NL, numberOfLandmarks);
            
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

            NP = l3.trianglebot.SystemModel.NP;

            [obj.x(1:NP), gradFx, gradFv] = obj.systemModel.predictState(obj.x(1:NP), obj.u, dT);

            % Multiply the top left block for the platform state
            obj.P(1:NP,1:NP) = gradFx * obj.P(1:NP, 1:NP) * gradFx' + gradFv * obj.sigmaU * gradFv';

            % Do the platform landmark-prediction blocks
            obj.P(1:NP, NP+1:end) = gradFx * obj.P(1:NP, NP+1:end);
            obj.P(NP+1:end, 1:NP) = obj.P(1:NP, NP+1:end)';

            success = true;
        end

        function success = handleInitializationEvent(obj, event)
            obj.x = event.data;
            obj.P = event.covariance;
            obj.initialized = true;
            success = true;
        end

        % Handle a set of measurements of landmarks
        function handleSLAMObservationEvent(obj, event)
            assert(obj.stepNumber == event.eventGeneratorStepNumber)

            % Store useful values
            NL = l3.trianglebot.SystemModel.NL;
            NP = l3.trianglebot.SystemModel.NP;

            % Get the list of landmarks we know about
            knownLandmarkIDs = obj.landmarkIDStateVectorMap.keys();

            % Find the set of known landmarks in the observation set; note
            % this can be disabled for some activities
            [existingLandmarks, idx] = intersect(event.info, knownLandmarkIDs);

            if (obj.updateKnownLandmarks == false)
                idx = [];
            end

            for o = 1 : numel(idx)
                % Look up the landmark and figure out its position
                offset = lookup(obj.landmarkIDStateVectorMap, existingLandmarks(o));
                landmarkIdx = offset + (1:NL);

                % Comput the prediction and Jacobian
                [zPred, gradHx, gradHm, gradHw] = ...
                    obj.systemModel.predictSLAMObservation(obj.x(1:NP), ...
                    obj.x(landmarkIdx));
                HS = zeros(2, numel(obj.x));
                HS(:, 1:NP) = gradHx;
                HS(:, landmarkIdx) = gradHm;
                C = obj.P * HS';
                S = HS * C + gradHw * event.covariance() * gradHw';
                nu = event.data(:, idx(o)) - zPred;
                nu(2) = atan2(sin(nu(2)), cos(nu(2)));

                K = C / S;

                obj.x = obj.x + K * nu;
                obj.P = obj.P - K * S * K';
            end

            % The remaining observations are of new landmarks
            [newLandmarks, idx] = setdiff(event.info, existingLandmarks);
            for o = 1 : numel(newLandmarks)

                % Figure out the index for the new state estimate and
                % insert it into the look up table
                stateDim = length(obj.x);
                landmarkIdx = stateDim + (1:NL);
                obj.landmarkIDStateVectorMap = ...
                    insert(obj.landmarkIDStateVectorMap, newLandmarks(o), stateDim);

                % Use the inverse observation model to estimate the
                % landmark position and compute the Jacobian
                [m, gradGx, gradGw] = ...
                    obj.systemModel.predictLandmarkFromSLAMObservation(obj.x, event.data(:, idx(o)));

                % Expand the state with the initial estimate of the
                % landmark position
                obj.x(landmarkIdx) = m;

                % Add the predicted landmark covariance. We do this first
                % because it forces obj.P to be resized to the correct
                % dimension
                obj.P(landmarkIdx, landmarkIdx) = gradGx * obj.P(1:NP, 1:NP) * gradGx' ...
                    + gradGw * event.covariance() * gradGw';

                % Compute the cross correlation
                obj.P(landmarkIdx, 1:end-NL) = gradGx * obj.P(1:NP, 1:end-NL);

                % Copy over tranposed version
                obj.P(1:end-NL,landmarkIdx) = obj.P(landmarkIdx, 1:end-NL)';
            end
        end

        function success = handleGPSObservationEvent(obj, event)

            [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictGPSObservation(obj.x(1:2));

            % Expand to the full stateg
            HS = zeros(2, numel(obj.x));
            HS(:, 1:l3.trianglebot.SystemModel.NP) = gradHx;

            % Kalman Filter Update
            nu = event.data - zPred;
            C = obj.P * HS';
            S = HS * C + gradHw * event.covariance() * gradHw';
            W = C / S;
            obj.x = obj.x + W * nu;
            obj.P = obj.P - W * S * W';
            success = true;
        end

        function success = handleCompassObservationEvent(obj, event)

            [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictCompassObservation(obj.x(1:l3.trianglebot.SystemModel.NP));

            % Expand to the full state
            HS = zeros(1, numel(obj.x));
            HS(:, 1:l3.trianglebot.SystemModel.NP) = gradHx;

            % Kalman Filter Update
            nu = event.data - zPred;
            nu = atan2(sin(nu), cos(nu));
            C = obj.P * HS';
            S = HS * C + gradHw * event.covariance() * gradHw';
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
                [zPred, gradHx, gradHw] = ...
                obj.systemModel.predictBearingObservation(x(1:2), ...
                    sensor.position, sensor.orientation);

                % Expand to full state
                HS = zeros(1, numel(obj.x));
                HS(:, 1:l2s.trianglebot.SystemModel.NP) = gradHx;
                
                % Kalman Filter Update
                nu = event.data(s) - zPred;
                nu = atan2(sin(nu), cos(nu));
                C = P * HS';
                S = HS * C + gradHw * event.covariance() * gradHw';
                W = C / S;
                x = x + W * nu;
                P = P - W * S * W';
            end
            obj.x = x;
            obj.P = P;
            success = true;
        end

        function success = handleUpdateOdometryEvent(obj, event)
                assert(obj.stepNumber == event.eventGeneratorStepNumber);
                obj.u = event.data;
                obj.sigmaU = event.covariance;
                success = true;
            end

        function storeStepResults(obj)
            % Store the estimate for the future
            obj.timeStore(:, obj.stepNumber + 1) = obj.currentTime;
            obj.xStore(:, obj.stepNumber + 1) = obj.x(1:l3.trianglebot.SystemModel.NP);
            obj.PStore(:, obj.stepNumber + 1) = diag(obj.P(1:l3.trianglebot.SystemModel.NP, ...
                1:l3.trianglebot.SystemModel.NP));

        end
    end
end