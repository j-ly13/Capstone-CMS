classdef PRM_Path
    %PRM_Pather Light Class that interfaces with the PRM
    %   Detailed explanation goes here
    
    properties
        mapInflated
        trackWidth = 1;
    end
    
    methods
        function obj = set.mapInflated(obj,map)
            obj.mapInflated = copy(map);
            inflate(obj.mapInflated, obj.trackWidth/2);
        end
        
        function path = pathFinder(obj, trackWidth,startLocation, endLocation)
            prm = robotics.PRM(obj.mapInflated);
            prm.ConnectionDistance = 10;
            prm.NumNodes = 1000;

            path = findpath(prm, startLocation, endLocation);


            while isempty(path)
                % No feasible path found yet, increase the number of nodes
                prm.NumNodes = prm.NumNodes + 10;

                % Use the |update| function to re-create the PRM roadmap with the changed
                % attribute
                update(prm);

                % Search for a feasible path with the updated PRM
                path = findpath(prm, startLocation, endLocation);
            end

            showPRM();
        end
        
        function showPRM()
            figure()
            show(prm);
        end
    end
end

