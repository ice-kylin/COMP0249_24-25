% This is the basic element which is stored in a hypergraph. It basically
% stores just an ID, an optional name, and information to show if it has
% been inserted into a graph.
%
% It should not be instantiated directly.


classdef HyperGraphElement < handle
    
    
    properties(Access = protected)
        
        % The graph the element is regstered with
        owningGraph;
        
        % The id of the vertex. This is of type int64
        elementId;
        
        % The name of this object
        elementName;
        
        % Flag shows if validated
        validated;
    end
    
    methods(Access = protected)
        
        function obj = HyperGraphElement()
            obj.owningGraph = [];
            obj.validated = false;
        end
        
    end
    
    methods(Access = public, Sealed = true)
        
        % Get the name of the vertex
        function name = name(obj)
            name = obj.elementName;
        end
        
        % Get the numerical ID of the vertex
        function id = id(obj)
            id = obj.elementId;
        end
        
        % The graph the vertex is registered with
        function graph = graph(obj)
            graph = obj.owningGraph;
        end
        
        function clearValidated(obj)
            obj.validated = false;
        end
    end
    
    methods(Access = public, Abstract)
        
        % Make sure the element is valid. The definition of validity is
        % different for vertices and edges.
        validate(obj);
        
    end
    
    methods(Access = protected)
        
        % Set the ID of the vertex; you can only do this if the vertex has
        % not been registered with a graph
        function setId(obj, newElementId)
            
            % Assume we can cast okay
            newElementId = int64(newElementId);
            
            % If no ID has been assigned, we can assume the vertex hasn't
            % been registered with a graph
            if (isempty(obj.elementId) == true)
                obj.elementId = newElementId;
                return;
            end
            
            % Check we haven't been registered with a graph already
            assert((obj.registered == false), ...
                'g2o::hypergraphelement::changeidafterregistration', ...
                ['Attempt to change the id of an element that has ' ...
                'already been added to a graph; oldID=%d, newID=%d'], ...
                obj.elementId, newElementId);
            
            % Now change the vertex ID
            obj.elementId = newElementId;
            
        end
    end
    
    % These methods are only accessible by the hypergraph
    
    methods(Access = {?g2o.core.HyperGraph})
        
        function setGraph(obj, owningGraph)
            obj.owningGraph = owningGraph;
        end
        
        function clearGraph(obj)
            obj.owningGraph = [];
        end
    end
end