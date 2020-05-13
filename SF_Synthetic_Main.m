% Eatron Technologies
% Multi-object tracking using synthetic radar and vision detections
% Version 2:
%  - Waypoint generation for each vehicle is done using the road information
%  - Alternatively, a simple scenario ("SimpleScenario.mat") can be used instead of waypoint generation
%    created in Driving Scenario Designer tootlbox
%  - Sensors configuration: 
%     1) Mercedes Benz Bertha (Dickmann et. al. 2015)
%     2) A simple configuration used in MATLAB synthetic scenario sensor fusion example.
%  - Clustering detections are performed wrt the vehicle diagonal instead of the
%    vehcile length

clear all
close all
clc

%% Parameters

% Assignment gate value
AssignmentThreshold = 30;        % The higher the Gate value, the higher the likelihood that every track...
                                 % will be assigned a detection.

% M/N initiation parameters
% The track is "confirmed" if after N consecutive updates at
% least M measurements are assigned to the track after the track initiation.
N = 5;
M = 4;

% Elimination threshold: The track will be deleted after EliminationTH # of updates without 
% any measurement update
EliminationTH = 10; % updates

% Measurement Noise
R = [22.1 0 0 0
     0 2209 0 0
     0 0 22.1 0
     0 0 0 2209];

% Process noise
Q= 7e-1.*eye(4);

% Performance anlysis parameters:
XScene = 80;
YScene = 40;
% PerfRadius is defined after scenario generation

%% Generate the Scenario

% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;  % seconds
SensorsSampleRate   = 0.1;  % seconds

EgoSpeed = 25; % m/s

%% Simple Scenario (Choice #1)

% Load scenario road and extract waypoints for each lane
Scenario = load('SimpleScenario.mat');
WPs{1} = Scenario.data.ActorSpecifications(2).Waypoints;
WPs{2} = Scenario.data.ActorSpecifications(1).Waypoints;
WPs{3} = Scenario.data.ActorSpecifications(3).Waypoints;

road(scenario, WPs{2}, 'lanes',lanespec(3));

% Ego vehicle (lane 2)
egoCar = vehicle(scenario, 'ClassID', 1);
egoWPs = circshift(WPs{2},-8);
path(egoCar, egoWPs, EgoSpeed);

% Car1 (passing car in lane 3)
Car1 = vehicle(scenario, 'ClassID', 1);
Car1WPs = circshift(WPs{1},0);
path(Car1, Car1WPs, EgoSpeed + 5);

% Car2 (car in lane 1)
Car2 = vehicle(scenario, 'ClassID', 1);
Car2WPs = circshift(WPs{3},-15);
path(Car2, Car2WPs, EgoSpeed -5);

% Ego follower (lane 2)
Car3 = vehicle(scenario, 'ClassID', 1);
Car3WPs = circshift(WPs{2},+5);
path(Car3, Car3WPs, EgoSpeed);

% Car4 (stopped car in lane 1)
Car4 = vehicle(scenario, 'ClassID', 1);
Car4WPs = circshift(WPs{3},-13);
path(Car4, Car4WPs, 1);

ActorRadius = norm([Car1.Length,Car1.Width]);
%---------------------------------------------------------------------------------------------
%% Waypoint generation (Choice #2)

% % Load scenario road and extract waypoints for each lane
% WPs = GetLanesWPs('Scenario3.mat');
% % Define road wtr the middle lane waypoints
% road(scenario, WPs{2}, 'lanes',lanespec(3));
% %%%%%%%%%%%% BE CAREFUL OF LANESPACE(3) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Ego vehicle (lane 2)
% egoCar = vehicle(scenario, 'ClassID', 1);
% path(egoCar, WPs{2}, EgoSpeed); % On right lane
% 
% % Car1 (passing car in lane 3)
% Car1 = vehicle(scenario, 'ClassID', 1);
% WPs{1} = circshift(WPs{1},20);
% path(Car1, WPs{1}, EgoSpeed + 2);
% 
% % Car2 (slower car in lane 1)
% Car2 = vehicle(scenario, 'ClassID', 1);
% WPs{3} = circshift(WPs{3},-50);
% path(Car2, WPs{3}, EgoSpeed -5);

%---------------------------------------------------------------------------------------------

%% Create a Tracker
% Create a |<matlab:doc('multiObjectTracker') multiObjectTracker>| to track
% the vehicles that are close to the ego vehicle. The tracker uses the
% |initSimDemoFilter| supporting function to initialize a constant velocity
% linear Kalman filter that works with position and velocity.
% 
% Tracking is done in 2-D. Although the sensors return measurements in 3-D,
% the motion itself is confined to the horizontal plane, so there is no
% need to track the height.
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

%% Define Sensors and Bird's Eye Plot
sensors = SensorsConfig(egoCar,SensorsSampleRate);

BEP = createDemoDisplay(egoCar, sensors);
BEP1 = createDemoDisplay(egoCar, sensors);

%% Fusion Loop for the scenario

Tracks = [];
count = 0;
toSnap = true;
TrackerStep = 0;
time0 = 0;
currentStep = 0;
Performance.Actors.Ground  = [];
Performance.Actors.EATracks = [];
Performance.Actors.MATracks = [];
Performance.MeanDistance.EA = [];
Performance.MeanDistance.MA = [];
Performance.GhostActors.EA = [];
Performance.GhostActors.MA = [];
while advance(scenario) %&& ishghandle(BEP.Parent)    
    currentStep = currentStep + 1;
    % Get the scenario time
    time = scenario.SimulationTime;
    
    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,length(sensors));
    for i = 1:length(sensors)
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
            end
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end
        
    
    % Update the tracker if there are new detections
    if any(isValidTime)
        TrackerStep = TrackerStep + 1;

%----------------------------------------------------------------------------------------------
%-----------------------------------MATLAB Tracker----------------------------------------------
%----------------------------------------------------------------------------------------------

        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks1 = updateTracks(tracker, detectionClusters, time);
        
%----------------------------------------------------------------------------------------------
%-----------------------------------Eatron Tracker----------------------------------------------
%----------------------------------------------------------------------------------------------
        
        %% Cluster Detections
        VehicleDim = [sensors{1}.ActorProfiles.Length, sensors{1}.ActorProfiles.Width,...
                      sensors{1}.ActorProfiles.Height];
        [DetectionClusters] = ClusterDetections(detections, VehicleDim);
        %% Fetch Measurements
        Measurements = FetchMeasurements(DetectionClusters);

       %% Tracker Kalman Prediction
        % Predict the tracks states from previous step and propagate them to the current
        % time-step
        time1 = scenario.SimulationTime;
        Fusion_dt = time1 - time0;
        time0 = time1;
        for i = 1:size(Tracks,2)
            [Tracks(i).State, Tracks(i).StateCovariance] = KF_Predict(...
             Tracks(i).State, Tracks(i).StateCovariance,Q,Fusion_dt);
        end
        %% Tracker Data Association
        % Calculate the distance btw measured objects (detections) and tracks
        [Tracks] = DataAssoc(Tracks, Measurements,TrackerStep,AssignmentThreshold,N,R);
        
         %% Tracker Kalman Update     
        for i = 1:size(Tracks,2)
            if Tracks(i).Assigned
                [Tracks(i).State,Tracks(i).StateCovariance,Tracks(i).Sk] = KF_Correct(...
                 Tracks(i).State,Tracks(i).StateCovariance,Tracks(i).Measurement,R);
            end
        end
        
         %% Tracker Management
       [confirmedTracks,Tracks] = TrackManagement(Tracks,N,M,EliminationTH);
        
        %% Plot

        % Update bird's-eye plot
          updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
          updateBEP(BEP1, egoCar, detections, confirmedTracks1, positionSelector, velocitySelector);

        %% Performance Metrics
        
        % Calculate the performance indices for each tracker
        [EAPerformanceIndices] = PerfomanceCalculation(ta,confirmedTracks,XScene,YScene,ActorRadius);
        [MAPerformanceIndices] = PerfomanceCalculation(ta,transpose(confirmedTracks1),XScene,YScene,ActorRadius);
        
        % Performance metric 1) # of tracked and ground vehicles in the
        % scene at the current step
        Performance.Actors.Ground    = [Performance.Actors.Ground;  EAPerformanceIndices.NoOfActorsInScene];
        Performance.Actors.EATracks = [Performance.Actors.EATracks; EAPerformanceIndices.NoOfTracksInScene];
        Performance.Actors.MATracks = [Performance.Actors.MATracks; MAPerformanceIndices.NoOfTracksInScene];
        
        % Performance metric 2) Mean distance of Actors in the scene wrt the
        % associated tracks using PerRadius
        Performance.MeanDistance.EA = [Performance.MeanDistance.EA; EAPerformanceIndices.MeanDistance];
        Performance.MeanDistance.MA = [Performance.MeanDistance.MA; MAPerformanceIndices.MeanDistance];

        % Performance metric 3) # of ghost vehicles
        % Ghost vehicle: An actor that no track is asigned to within the
        % ghost region (PerRadius) around the vehicle at the currecnt step
        Performance.GhostActors.EA = [Performance.GhostActors.EA; EAPerformanceIndices.GhostActors];
        Performance.GhostActors.MA = [Performance.GhostActors.MA; MAPerformanceIndices.GhostActors];
    end

end

%% Performance Plot
figure;
    plot(Performance.Actors.MATracks,'b')
    hold on
    plot(Performance.Actors.EATracks,'r')
    plot(Performance.Actors.Ground,'--k')
    title('Performance Index 1: # of tracks and actors')
    xlabel('Step')
    ylabel('# of vehicles in the scene')
    legend('MATLAB Tracker','Eatron Tracker','Ground Truth')
    
figure;
    plot(Performance.MeanDistance.MA,'b-*')
    hold on
    plot(Performance.MeanDistance.EA,'r-*');
    title('Performance Index 2: Average distance btw tracks and actors')
    xlabel('Step')
    ylabel('Avrerage distance (m)')
    legend('MATLAB Tracker','Eatron Tracker')

figure;
    plot(Performance.GhostActors.MA,'b-*')
    hold on
    plot(Performance.GhostActors.EA,'r-o')
    title('Performance Index 3: # of ghost actors')
    xlabel('Step')
    ylabel('# of ghost actors in the scene')
    legend('MATLAB Tracker','Eatron Tracker')
    
%% Summary
% This example shows how to generate a scenario, simulate sensor detections, 
% and use these detections to track moving vehicles around the ego vehicle.
% 
% You can try to modify the scenario road, or add or remove vehicles. You 
% can also try to add, remove, or modify the sensors on the ego vehicle, or modify 
% the tracker parameters.

%% Supporting Functions
% *|initSimDemoFilter|*
% 
% This function initializes a constant velocity filter based on a detection.
function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

%%%
% *|clusterDetections|*
% 
% This function merges multiple detections suspected to be of the same
% vehicle to a single detection. The function looks for detections that are
% closer than the size of a vehicle. Detections that fit this criterion are
% considered a cluster and are merged to a single detection at the centroid
% of the cluster. The measurement noises are modified to represent the
% possibility that each detection can be anywhere on the vehicle.
% Therefore, the noise should have the same size as the vehicle size.
% 
% In addition, this function removes the third dimension of the measurement 
% (the height) and reduces the measurement vector to [x;y;vx;vy].
function detectionClusters = clusterDetections(detections, vehicleSize)
N = numel(detections);
distances = zeros(N);
for i = 1:N
    for j = i+1:N
        if detections{i}.SensorIndex == detections{j}.SensorIndex
            distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
        else
            distances(i,j) = inf;
        end
    end
end
leftToCheck = 1:N;
i = 0;
detectionClusters = cell(N,1);
while ~isempty(leftToCheck)    
    % Remove the detections that are in the same cluster as the one under
    % consideration
    underConsideration = leftToCheck(1);
    clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
    detInds = leftToCheck(clusterInds);
    clusterDets = [detections{detInds}];
    clusterMeas = [clusterDets.Measurement];
    meas = mean(clusterMeas, 2);
    meas2D = [meas(1:2);meas(4:5)];
    i = i + 1;
    detectionClusters{i} = detections{detInds(1)};
    detectionClusters{i}.Measurement = meas2D;
    leftToCheck(clusterInds) = [];    
end
detectionClusters(i+1:end) = [];

% Since the detections are now for clusters, modify the noise to represent
% that they are of the whole car
for i = 1:numel(detectionClusters)
    measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
    measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
    detectionClusters{i}.MeasurementNoise = measNoise;
end
end

%%% 
% *|createDemoDisplay|*
% 
% This function creates a three-panel display:
% 
% # Top-left corner of display: A top view that follows the ego vehicle.
% # Bottom-left corner of display: A chase-camera view that follows the ego vehicle.
% # Right-half of display: A <matlab:doc('birdsEyePlot') bird's-eye plot> display.
function BEP = createDemoDisplay(egoCar, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top    

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 180, 'ViewLocation', [0 0], 'ViewPitch', 90);
    
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');
    
    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 80;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    
    % Plot the coverage areas for long-range radars
    for i = 1:4
        cap = coverageAreaPlotter(BEP,'FaceColor',[1 0.4 0.6],'EdgeColor',[1 0.4 0.6]);
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end
    
    % Plot the coverage areas for mid-range radars
    for i = 5:8
        cap = coverageAreaPlotter(BEP,'FaceColor',[1 0.6 0.6],'EdgeColor',[1 0.6 0.6]);
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end
    
    % Plot the coverage areas for short-range radars
    for i = 9:12
        cap = coverageAreaPlotter(BEP,'FaceColor',[0.8 0.4 0.4],'EdgeColor',[0.8 0.4 0.4]);
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end
    
%     Plot the coverage areas for vision sensors
    for i = 13:14
        cap = coverageAreaPlotter(BEP,'FaceColor',[0.2 0.4 1],'EdgeColor',[0.2 0.4 1]);
        plotCoverageArea(cap, sensors{i}.SensorLocation,...
            sensors{i}.MaxRange, sensors{i}.Yaw, 45);
    end
    
    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');
    
    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');
    
    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');
    
    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);
    
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);
    
    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end

%%% 
% *|updateBEP|*
% 
% This function updates the bird's-eye plot with road boundaries,
% detections, and tracks.
function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);
    
    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
    
    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);    
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 12 % Vision detections            
            isRadar(i) = false;
        end        
    end
    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));    
    
    % Prepare and update tracks display
    if ~isempty(confirmedTracks)
        trackIDs = {confirmedTracks.TrackID};
        labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
        [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
        tracksVel = getTrackVelocities(confirmedTracks, vsel);
        plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
    end
end