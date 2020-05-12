function [Tracks] = DataAssoc(Tracks, Measurements,currentStep,AssignmentThreshold,ConsecNum,R)
% Associate the tracks to the measurements

% Measurementt matrix
H = eye(4);

M = size(Tracks,2);                     % Number of assigned/unassigned tracking objects
N = size(Measurements,2);               % Number of measurements
CostMat = zeros(M,N);                   % Association cost matrix

% pCostOfNonAssignment A scalar value that defines the cost if a
% track is not assigned or a detection is not assigned. Since this
% cost is incurred twice when a detection falls outside of the a
% track gate, it is half the AssignmentThreshold
costOfNonAssignment = AssignmentThreshold/2;

%% Innovation covariance calculation
for i=1:size(Tracks,2)
   Tracks(i).Sk = H*Tracks(i).StateCovariance*transpose(H) + R; 
end

%% Hungarian assignment algorithm
for i=1:M
    for j=1:N
        %  - Synchronized sensors (unless we would not use Xpred (state) because we
        %    could not be sure about the value of Kalman_dt at this step)
        %  - Xpred is used (state) instead of the Xest because both of them are
        %    calculted in the previous step thus Xpred is the correst
        %    estimation for the state vector in the current step.
        Y            = Measurements(:,j);
        % The predicted output from KF prediction step in previous step
        Yhat         = H*Tracks(i).State;
        % Mahalanobis distance:
        dist_ij      = Y - Yhat;
        Pk           = Tracks(i).StateCovariance;
%         Sk = H*Pk*transpose(H) + R;               % Innovation Covariance
%         Pk = diag([1 100 1 100])*Pk;
        costVal = transpose(dist_ij)*inv(Pk)*dist_ij;
        
        if costVal > AssignmentThreshold
            CostMat(i,j) = Inf;
        else
            CostMat(i,j) = costVal;
        end

    end
end

if isempty(Measurements)
    disp('Error. No measurments available for this step')
end

[AssignIdxMat,UnassignIdxMat,UassignDetectionMat] = ...
    assignDetectionsToTracks(CostMat,costOfNonAssignment);

%% Assign measurements to existing tracks
% Inputs: Measurements - Tracks
% Output: Updated Tracks (assigned)

for i=1:size(AssignIdxMat,1)
    trackIdx                            = AssignIdxMat(i,1);
    detectIdx                           = AssignIdxMat(i,2);
    
    % Assignment flag
    Tracks(trackIdx).Assigned           = 1;
    % Track age
    Tracks(trackIdx).Age                = currentStep - Tracks(i).InitStep;
    % Update measurement
    Tracks(trackIdx).Measurement        = Measurements(:,detectIdx); 
    % Save the assignment in history:
        % Shift vector to left by 1
        Tracks(trackIdx).AssignHis      = circshift(Tracks(trackIdx).AssignHis,-1);
        % Save the assignemtn flag to the last element
        Tracks(trackIdx).AssignHis(end) = 1;
end

%% Unassigned tracks

for i=1:length(UnassignIdxMat)
    trackIdx                             = UnassignIdxMat(i);
    % Assignment flag
    Tracks(trackIdx).Assigned            = 0;
    % Track age
    Tracks(trackIdx).Age                 = currentStep - Tracks(i).InitStep;
    Tracks(trackIdx).UnassignmentCounter = Tracks(trackIdx).UnassignmentCounter + 1;
    % Save the unassignment in history:
        % Shift vector to left by 1
        Tracks(trackIdx).AssignHis = circshift(Tracks(trackIdx).AssignHis,-1);
        % Save the unassignemtn flag to the last element
        Tracks(trackIdx).AssignHis(end) = 0;
end

% Add next: if there is no measurement, all tracks become unassigned.

%% New tracks
%  Set unassigned detections as new tracks

n = size(Tracks,2);
for i=1:length(UassignDetectionMat)
    trackIdx                     = n+i;
    MeasIdx = UassignDetectionMat(i);
    % Track initialization:
    
    Tracks(trackIdx).TrackID                   = 0;
    % Initialize the track state with the first measurement
    Tracks(trackIdx).State              = [Measurements(1,MeasIdx)        % x
                                           Measurements(2,MeasIdx)        % vx         
                                           Measurements(3,MeasIdx)        % y
                                           Measurements(4,MeasIdx)];      % vy 
    Tracks(trackIdx).StateCovariance     = eye(4);
    Tracks(trackIdx).Sk                  = eye(4);
    % Track measurement
    Tracks(trackIdx).Measurement         = Measurements(:,MeasIdx);
    % Record the step number where the track initialized
    Tracks(trackIdx).InitStep            = currentStep;
    % Track age
    Tracks(trackIdx).Age                 = 0;
    % Is tarck assigned (1) or unassigned (0)
    Tracks(trackIdx).Assigned            = 0;
    % Unassignment counter
    Tracks(trackIdx).UnassignmentCounter = 0;
    % Record a history of the track up to Nconsec # of consecutive steps
    Tracks(trackIdx).AssignHis           = zeros(ConsecNum,1);
    % Confirmed (true) or unconfirmed (false)
    Tracks(trackIdx).Confirmed = false;
end

end










