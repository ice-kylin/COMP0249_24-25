classdef SystemModel < handle

    properties(Access = public, Constant)
        % Platform state dimension
        NP = 2;
        
        % Landmark dimension
        NL = 2;
    end


    properties(GetAccess = public)

        % Continuous time process model
        Fc;
        Fd;
        Bc;
        Bd;

        % Continuous time process noise model
        Qc;
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

        cachedDT;

    end

    properties(Access = protected)
        config;
        perturbWithNoise;
    end

    methods(Access = public)

        function obj = SystemModel(config, perturbWithNoise)
            obj.config = config;

            if (nargin == 1)
                obj.perturbWithNoise = false;
            else
                obj.perturbWithNoise = perturbWithNoise;
            end
            
            obj.setupModels();

            obj.cachedDT = NaN;
        end

        function [x, F, Q] = predictState(obj, x, u, dT)

            % obj.Fd = [1 0;0 1];
            % obj.Bd = [dT 0;0 dT];
            % obj.Qd = obj.config.platform.sigmaQ^2 * dT^2/2;
            % 
            if (dT ~= obj.cachedDT)
                [obj.Fd, obj.Qd, obj.Bd] = ebe.utils.continuousToDiscrete(obj.Fc, ...
                    obj.Qc, obj.Bc, dT);
                if (obj.perturbWithNoise == true)
                    obj.QdSqrtm = sqrtm(obj.Qd);
                end
                obj.cachedDT = dT;
            end

            pIdx = 1:l2.dotbot.SystemModel.NP;
            x(pIdx) = obj.Fd * x(pIdx) + obj.Bd * u;

            if (obj.perturbWithNoise == true)
                x(pIdx) = x(pIdx) + obj.QdSqrtm * randn(l2.dotbot.SystemModel.NP, 1);
            end

            if (nargout == 3)
                F = eye(length(x));
                F(pIdx, pIdx) = obj.Fd;
                Q = zeros(length(x));
                Q(pIdx, pIdx) = obj.Qd;
            end
        end

        function [z, H, R] = predictGPSObservation(obj, x)

           z = obj.HGPS * x(1:l2.dotbot.SystemModel.NP);

            if (obj.perturbWithNoise == true)
                z = z + obj.RGPSSqrtm * randn(size(z));
            end

            if (nargout == 3)
                H = zeros(2, length(x));
                H(:, 1:l2.dotbot.SystemModel.NP) = obj.HGPS;
                R = obj.RGPS;
            end
        end

        function [z, H, R] = predictBearingObservation(obj, x, sensorXY, sensorTheta)

            dx = x(1) - sensorXY(1);
            dy = x(3) - sensorXY(2);
            z = atan2(dy, dx) - sensorTheta * pi / 180;

            if (obj.perturbWithNoise == true)
                z = z + obj.RBearingSqrt * randn(size(z));
            end

            z = atan2(sin(z), cos(z));

            if (nargout == 3)
                d = dx^2 + dy^2;
                H = [dy/d 0 dx/d 0];
                R = obj.RBearing;
            end
        end

        function [z, Hp, Hl, R] = predictSLAMObservation(obj, x, lXY)

            z = lXY - x(1:2);

            if (obj.perturbWithNoise == true)
                z = z + obj.RSLAMSqrt * randn(size(z));
            end
            if (nargout == 4)
                Hp = -eye(2);
                Hl = eye(2);
                R = obj.RSLAM;
            end

        end

    end

    methods(Access = protected)

        function setupModels(obj)

            % Set up the continuous time process model
            obj.Fc=zeros(2);

            obj.Qc = eye(2) * obj.config.platform.controller.odomSigma^2;

            obj.Bc = [1 0;0 1];
                        
            if (isfield(obj.config, 'map') == false)
                return
            end
            
            if (isfield(obj.config.map, 'sensors') == false)
                return
            end

            % Set up the observation
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