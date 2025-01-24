classdef SimulatorSystemModel < handle

    properties(GetAccess = public)

        % Process noise sample
        Qd;
        QdSqrtm;

        % Observation model
        HGPS;

        % GPS
        RGPS;
        RGPSSqrtm;

        % Bearing
        RBearing;
        RBearingSqrt;

        % SLAM
        RSLAM;
        RSLAMSqrt;

    end

    properties(Access = protected)
        config;
        perturbWithNoise;
    end

    methods(Access = public)

        function obj = SimulatorSystemModel(config, perturbWithNoise)
            obj.config = config;

            if (nargin == 1)
                obj.perturbWithNoise = false;
            else
                obj.perturbWithNoise = perturbWithNoise;
            end
            
            obj.setupModels();
        end

        function [x, F, Q] = predictState(obj, x, u, dT)

            sDT = u(1) * dT;

            x(1) = x(1) + sDT * cos(x(3));
            x(2) = x(2) + sDT * sin(x(3));
            x(3) = x(3) + dT * u(2);
            
            x(3) = atan2(sin(x(3)), cos(x(3)));

            if (nargout == 3)
                F = eye(3);
                F(1, 3) = -sDT * sin(x(3));
                F(2, 3)= sDT * cos(x(3));
                Q = obj.Qd;
            end
        end

        function [z, H, R] = predictRBObservation(obj, x, lXY)

            dXY = lXY - x(1:2);
            d2=sum(dXY.^2);
            d=sqrt(d2);
            z = [d;
                atan2(dXY(2), dXY(1)) - x(3)];

            if (obj.perturbWithNoise == true)
                z = z + obj.RRBSqrtm * randn(size(z));
            end

            z(2) = atan2(sin(z(2)), cos(z(2)));

            if (nargout == 3)
                H=[-dXY(1)/d -dXY(2)/d 0;
                dXY(2)/d2 -dXY(1)/d2 -1];
                R = obj.RRB;
            end

        end

        function [z, Hp, Hl, R] = predictSLAMObservation(obj, x, lXY)

            z = lXY - x(1:2);

            if (obj.perturbWithNoise == true)
                z = z + obj.RSLAMSqrt * randn(size(z));
            end
            if (nargout == 4)
                Hp = -[1 0 0 0;0 0 1 0];
                Hl = eye(2);
                R = obj.RSLAM;
            end

        end


    end

    methods(Access = protected)

        function setupModels(obj)
            % Set up the observation

            if (isfield(obj.config, 'map') == false)
                return
            end

            if (isfield(obj.config.map.sensors, 'gps'))
                obj.HGPS = [1 0 0 0;0 0 1 0];
                obj.RGPS = eye(2) * obj.config.map.sensors.gps.sigmaR^2;
    
                if (obj.perturbWithNoise == true)
                    obj.RGPSSqrtm = obj.config.map.sensors.gps.sigmaR;
                end
            end

            if (isfield(obj.config.map.sensors, 'bearing'))
                obj.RBearing = deg2rad(obj.config.map.sensors.bearing.sigmaR)^2;
                if (obj.perturbWithNoise == true)
                    obj.RBearingSqrt = deg2rad(obj.config.map.sensors.bearing.sigmaR);
                end
            end

            if (isfield(obj.config.map.sensors, 'slam'))
                obj.RSLAM = eye(2) * obj.config.map.sensors.slam.sigmaR^2;
                if (obj.perturbWithNoise == true)
                    obj.RSLAMSqrt = eye(2) * obj.config.map.sensors.slam.sigmaR;
                end
            end
        end
    end

end