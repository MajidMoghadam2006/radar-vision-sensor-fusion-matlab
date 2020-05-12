function [DetectionClusters] = ClusterDetections(Detections, VehicleDim)
% This function clusters detections from various sensors wrt the dimension
% of the vehicle.
% The measurement noise covariance matrix is also being midified

SizeThreshold = norm( [VehicleDim(1), VehicleDim(2)] );
N = size(Detections,1);
Distances = zeros(N);

for i = 1:N
    for j = i+1:N
            Distances(i,j) = norm(Detections{i}.Measurement(1:2) - ...
                                  Detections{j}.Measurement(1:2));
    end
end

DetectionClusters = cell(N,1);
leftToCheck = 1:N;
i = 0;
while ~isempty(leftToCheck)    
    % Remove the detections that are in the same cluster as the one under
    % consideration
    underConsideration = leftToCheck(1);
    clusterInds = (Distances(underConsideration, leftToCheck) < SizeThreshold);
    detInds = leftToCheck(clusterInds);
    clusterDets = [Detections{detInds}];
    clusterMeas = [clusterDets.Measurement];
    meas = mean(clusterMeas, 2);
    meas2D = [meas(1:2);meas(4:5)];
    i = i + 1;
    DetectionClusters{i} = Detections{detInds(1)};
    DetectionClusters{i}.Measurement = meas2D;
    leftToCheck(clusterInds) = [];    
end
DetectionClusters(i+1:end) = [];

% Since the detections are now for clusters, modify the noise to represent
% that they are of the whole car
for i = 1:numel(DetectionClusters)
    measNoise(1:2,1:2) = SizeThreshold^2 * eye(2);
    measNoise(3:4,3:4) = eye(2) * 100 * SizeThreshold^2;
    DetectionClusters{i}.MeasurementNoise = measNoise;
end

end

