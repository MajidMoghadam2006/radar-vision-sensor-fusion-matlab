function [WPs] = GetLanesWPs(sc)
% Receive the saved road scenario and generates the required waypoints for
% each lane
% NOTE: This function only works for the connected-end scenarios

% Scenario = load('Scenario3.mat');
Scenario = load(sc);
NoOfLanes = Scenario.data.RoadSpecifications.Lanes.NumLanes;
roadCenters = Scenario.data.RoadSpecifications.Centers(:,1:2);

scenario = drivingScenario;
road(scenario, roadCenters, 'lanes',lanespec(NoOfLanes));

% Prepare road boundaries and widths to generate waypoints
rb = roadBoundaries(scenario);
outerBoundary = rb{1};
innerBoundary = rb{2};
innerBoundary = transpose(innerBoundary);
innerBoundary = fliplr(innerBoundary);
innerBoundary = transpose(innerBoundary);

WPs = {};
j=1;
for i = 1:NoOfLanes
    WPs{i} = ( (j)*innerBoundary(:,1:2) + ...
               (2*NoOfLanes-j-1)*outerBoundary(:,1:2)) /(2*NoOfLanes);
    j = j + 2;
end

end

