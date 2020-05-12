function [PerformanceIndices] = PerfomanceCalculation(GT,Tracks,XScene,YScene,ActorRadius)
% This function caculates the performance indices of the confirmed tracks

% Scene: A virtual rectangular region around the ego car normally
% with (2*XScene,2*YScene) dimensions

% PerformanceIndices.NoOfActorsInScene:
% # of the ground truth actors in the scene

% PerformanceIndices.NoOfTracksInScene:
% # of tracks in the scene

% PerformanceIndices.MeanDistance:
% The average distance btw the tracks inside the ActorRadius and the ground
% truth actor

% PerformanceIndices.GhostActors:
% # of actors that no track is associated to within the ActorRadius region
% around the actor

        % Record the ground vehicles in the scene
        j=1;
        NoOfActorsInScene = 0;
        for i=1:size(GT,2)
            if (abs(GT(i).Position(1)) <= XScene) && ...
               (abs(GT(i).Position(2)) <= YScene)
                SceneGroundCars(j) = GT(i);
                j = j + 1;
                NoOfActorsInScene = NoOfActorsInScene + 1;
            end
        end
        
        % Record the tracks in the scene found by Eatron tracker in
        % SceneTracks and associate the tracks to actors that are 
        % closer than PerRadius
        j=1;
        NoOfTracksInScene = 0;
        TrackGroundAssocVec = [];
        for i=1:size(Tracks,2)
            if (abs(Tracks(i).State(1)) <= XScene) && ...
               (abs(Tracks(i).State(3)) <= YScene)
                SceneTracks(j) = Tracks(i);
                NoOfTracksInScene = NoOfTracksInScene + 1;
                % Is this track asociated with an actor
                for k=1:size(SceneGroundCars,2)
                    distance = norm( SceneGroundCars(k).Position(1:2) - ...
                                     transpose(SceneTracks(j).State([1,3])) );
                    if distance <= ActorRadius
                        % The k_th SceneGroundCars actor is associated to
                        % the j_th SceneGroundCars track.
                        TrackGroundAssocVec = [TrackGroundAssocVec; k,j,distance];
                    end
                end
            j = j + 1;
            end
        end

        % Performance metric 1) # of tracked and ground vehicles in the
        % scene at the current step
        PerformanceIndices.NoOfActorsInScene = NoOfActorsInScene;
        PerformanceIndices.NoOfTracksInScene = NoOfTracksInScene;
        
        % Performance metric 2) Mean distance of Actors in the scene wrt the
        % associated tracks using PerRadius
        if numel(TrackGroundAssocVec) ~= 0
            MeanDistance = mean(TrackGroundAssocVec(:,3));
        else
            MeanDistance = NaN;
        end
        PerformanceIndices.MeanDistance = MeanDistance;

        % Performance metric 3) # of ghost vehicles
        % Ghost vehicle: An actor that no track is asigned to within the
        % ghost region (PerRadius) around the vehicle at the currecnt step
        if numel(TrackGroundAssocVec) ~= 0
                GhostActors = 0;
            for i=1:size(SceneGroundCars,2)
                if sum(TrackGroundAssocVec(:,1) == i) == 0
                   GhostActors = GhostActors + 1;
                end
            end
        else
            % If there is not any associated track, all actors are ghost actors
            GhostActors = size(SceneGroundCars,2);
        end
        PerformanceIndices.GhostActors = GhostActors;
end

