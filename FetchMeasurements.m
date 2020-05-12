function [Measurements] = FetchMeasurements(Detections)
%This function fetchs the measurements from the detections and transforms
%them to the required format for KF
% Detections.Measurements = [x y vx vy]'
% Measurements = [x vx y vy]'

% % Raw detections
% H = [1 0 0 0 0 0
%      0 0 0 1 0 0
%      0 1 0 0 0 0
%      0 0 0 0 1 0];

% clustered detections
H = [1 0 0 0        % x
     0 0 1 0        % vx
     0 1 0 0        % y
     0 0 0 1];      % vy

 Measurements = zeros(4,length(Detections));
 for i=1:length(Detections)
     Measurements(:,i) = H*Detections{i}.Measurement;
 end


end

