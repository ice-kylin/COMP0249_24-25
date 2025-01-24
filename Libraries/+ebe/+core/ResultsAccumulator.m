classdef ResultsAccumulator < ebe.core.Component

    % This class is called once per step and is used to store information
    % on the performance of each estimator

    properties(Access = protected)

        eventGenerator;

        estimators;

    end

    methods(Access = public)

        function setEventGeneratorAndEstimators(obj, eventGenerator, estimators)
            obj.eventGenerator = eventGenerator;
            obj.estimators = estimators;
        end
    end

    methods(Access = public, Abstract)

        collectResults(obj);

    end

end
