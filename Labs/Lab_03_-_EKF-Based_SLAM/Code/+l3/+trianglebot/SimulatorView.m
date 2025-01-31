classdef SimulatorView < ebe.graphics.EventGeneratorView

    properties(Access = protected)

        % The ground truth geometry
        groundTruthDrawer;

        % Obstacle geometry
        occluderGeometry;

        % Draw the observation mean and covariance
        gpsZRDrawer;

        % Geometry for the bearing measurements
        bearingObservationGeometry;

        slamObservationGeometry;

        % Information about the axes
        drawAxes;

        % If set to true, the plot is always centered on the ground truth
        % position
        centreAxes;

    end

    methods(Access = public)
        function obj = SimulatorView(config, eventGenerator)
            obj@ebe.graphics.EventGeneratorView(config, eventGenerator);
            obj.drawAxes = axis;
            obj.centreAxes = false;
        end

        function setCentreAxesOnTruth(obj, centreAxes)
            obj.centreAxes = centreAxes;
        end

        function start(obj)

            % Allocate the ground truth colour first
            xTrueColour = ebe.graphics.DistinguishableColours.assignColour('Ground Truth');

            % Now allocate the colour for the waypoints
            waypointColour = ebe.graphics.DistinguishableColours.assignColour('waypoints');

            % Draw the map if it's defined
            scenario = obj.eventGenerator.scenario();

            % Draw the waypoints (these go on the bottom)
            if (isfield(obj.config, 'platformTrajectory'))
                waypoints = obj.config.platformTrajectory.waypoints;
                plot(waypoints(:, 1), waypoints(:, 2), 'o', 'Color', waypointColour, 'MarkerSize', 4, 'LineWidth', 2);

                % To draw the connected line, we have to add the first
                % point back again
                numWayPoints = size(waypoints, 1);
                plot(waypoints([1:numWayPoints 1], 1), waypoints([1:numWayPoints 1], 2), 'Color', waypointColour, 'LineWidth', 2);
            end

            if (isempty(scenario) == false)
                % Draw the landmarks if present
                if (isfield(scenario, 'landmarks'))
                   obj.plotLandmarks(obj.eventGenerator.landmarks());
                end
    
                % Draw the different types of sensors if present
                if (isfield(scenario, 'sensors'))
                    if (isfield(scenario.sensors, 'gps'))
                       obj.plotGPSAndOccluders(scenario.sensors.gps);
                    end
                    if (isfield(scenario.sensors, 'bearing'))
                        obj.plotBearingSensors(scenario.sensors.bearing);
                    end
                    if (isfield(scenario.sensors, 'slam'))
                        obj.plotSLAMSensor(scenario.sensors.slam);
                    end
                end                
            end



            % Set up the drawing for the ground truth position
            obj.groundTruthDrawer = ebe.graphics.OrientedTriangleDrawer(xTrueColour);

            % Now 
        end

        function visualize(obj, events)

            % Update the ground truth position
            xTrue = obj.eventGenerator.xTrue();
            obj.groundTruthDrawer.update(xTrue);

            % Now check all the events and process
            for e = 1 : length(events)                
                event = events{e};
                if (strcmp(event.type, 'gps') == true)
                    obj.visualizeGPSObservation(event);
                elseif (strcmp(event.type, 'bearing') == true)
                    obj.visualizeBearingObservation(event);
                elseif (strcmp(event.type, 'slam') == true)
                    obj.visualizeSLAMObservations(event);
                end
            end

            % Reposition the axes if necessary
            if (obj.centreAxes == true)
                axes = obj.drawAxes();
                axes(1:2) = axes(1:2) + xTrue(1);
                axes(3:4) = axes(3:4) + xTrue(2);
                axis(axes);
            end
        end
    end

    methods(Access = protected)

        function plotGPSAndOccluders(obj, gps)

            % Draw the GPS observations
            zGPSColour = ebe.graphics.DistinguishableColours.assignColour('gps');            
            obj.gpsZRDrawer = ebe.graphics.MeanCovarianceDrawer(zGPSColour);

            if (isfield(gps, 'occluders') == false)
                return
            end

            numOccluders = numel(gps.occluders);

            obj.occluderGeometry = cell(numOccluders, 1);
            for i = 1:numOccluders
                occluder = gps.occluders(i);
                obj.occluderGeometry{i}=rectangle('Position', [occluder.x_min, occluder.y_min, ...
                    occluder.x_max - occluder.x_min, occluder.y_max - occluder.y_min], ...
                    'EdgeColor', 'k', 'FaceColor', [0.8, 0.8, 0.8, 0.7], 'LineWidth', 2);
            end
        end

        function plotBearingSensors(obj, bearing)

            % Draw the bearing sensor observations
            zBearingColour = ebe.graphics.DistinguishableColours.assignColour('bearing');            
            obj.bearingObservationGeometry = plot(NaN, NaN, 'Color', zBearingColour);

            for s = 1 : numel(bearing.sensors)
                sensor = bearing.sensors(s);

                % Extract sensor properties
                pos = sensor.position;
                orientation = sensor.orientation;
                range = sensor.detectionRange;
                angle = sensor.detectionAngle;

                % Compute circular arc points
                theta = linspace(orientation - angle / 2, orientation + angle / 2, 100) * pi / 180;
                arcX = pos(1) + range * cos(theta);
                arcY = pos(2) + range * sin(theta);

                % Complete the wedge by connecting to the sensor's position
                wedgeX = [pos(1), arcX, pos(1)];
                wedgeY = [pos(2), arcY, pos(2)];

                % Plot sensor wedge with transparency
                fill(wedgeX, wedgeY, [1, 0.8, 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');

                % Plot edges of the wedge
                line([pos(1), arcX(1)], [pos(2), arcY(1)], 'Color', 'r', 'LineWidth', 2);
                line([pos(1), arcX(end)], [pos(2), arcY(end)], 'Color', 'r', 'LineWidth', 2);

                % Plot circular arc
                plot(arcX, arcY, 'r', 'LineWidth', 2);
            end
        end

        function plotSLAMSensor(obj, ~)
            slamSensorColour = ebe.graphics.DistinguishableColours.assignColour('slam_sensor_colour');
            obj.slamObservationGeometry = plot(NaN, NaN, 'Color', slamSensorColour, 'LineWidth', 2);
        end

        function plotLandmarks(obj, landmarks)
            landmarkColour = ebe.graphics.DistinguishableColours.assignColour('slam_true_landmarks');
            plot(landmarks(1, :), landmarks(2, :), '+', 'Color', landmarkColour, 'MarkerSize', 7, 'LineWidth', 2);
        end

        function visualizeGPSObservation(obj, event)
            obj.gpsZRDrawer.update(event.data, event.covariance);
        end

        function visualizeBearingObservation(obj, event)
            map = obj.eventGenerator.map();
            numObservations = numel(event.info);
            bearing = event.data;
            xy = NaN(2, 3 * numObservations);
            for s = 1 : numObservations
                sensor = map.sensors.bearing.sensors(event.info(s));
                xy(:,3*s-2) = sensor.position;
                theta = bearing(s) + deg2rad(sensor.orientation);
                xy(:,3*s-1) = sensor.position + sensor.detectionRange * [cos(theta); sin(theta)];
            end
            set(obj.bearingObservationGeometry, 'XData', xy(1, :), 'YData', xy(2, :))
        end

        function visualizeSLAMObservations(obj, event)
            xTrue = obj.eventGenerator.xTrue();

            z = event.data;

            numObservations = numel(event.info);
            xy = NaN(2, 3 * numObservations);
            for s = 1 : numObservations
                beta = z(2,s) + xTrue(3);
                xy(:, 3*s-2) = xTrue(1:2);
                xy(:, 3*s-1) = xy(:, 3*s-2) + z(1, s) * [cos(beta);sin(beta)];
            end
            set(obj.slamObservationGeometry, 'XData', xy(1, :), 'YData', xy(2, :));
        end

    end

end