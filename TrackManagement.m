function [confirmedTracks,Tracks] = TrackManagement(Tracks,N,M,EliminationTH)

persistent IDCounter
if isempty(IDCounter)
    IDCounter = 1;
end

confirmedTracks = [];
cIdx=1;
for i=1:size(Tracks,2)
    %% M/N Initiation method:
    
    NumOfAssignments   = sum(Tracks(i).AssignHis == 1);
    NumOfUnassignments = sum(Tracks(i).AssignHis == 0);
    
    if NumOfAssignments >= M
        Tracks(i).Confirmed = true;
        % Mark the confirmed track with TrackID if the track is new
        if Tracks(i).TrackID == 0
            Tracks(i).TrackID = IDCounter;
            IDCounter         = IDCounter + 1;
        end
    end
    
    if NumOfUnassignments >= N
        Tracks(i).Confirmed = false;
    end
    
    %% Pack the confirmed tracks
    if Tracks(i).Confirmed
        confirmedTracks(cIdx).TrackID = Tracks(i).TrackID;
        confirmedTracks(cIdx).Age = Tracks(i).Age;
        confirmedTracks(cIdx).State = Tracks(i).State;
        confirmedTracks(cIdx).StateCovariance = Tracks(i).StateCovariance;
        cIdx = cIdx + 1; 
    end
        
end

%% Coasting tracks : Remove unassigned old tacks

RemoveIdx = [];
for i=1:size(Tracks,2)
   if Tracks(i).UnassignmentCounter >= EliminationTH
       RemoveIdx = [RemoveIdx i];
   end
end
Tracks(RemoveIdx) = [];

%% 
% if ~exist('confirmedTracks','var')
%     confirmedTracks = [];
% end

end

