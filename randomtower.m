simulationDuration = 5;
scene = radarScenario('StopTime', simulationDuration);
tp = theaterPlot('XLimits', [-360 360], 'YLimits', [-360 360], 'ZLimits', [0 120]);
view(3);
grid on;

timeOfArrival = [0  simulationDuration];
waypoints = [200 -100 10; 150 110 110];
trajectory = waypointTrajectory(waypoints,timeOfArrival);



target = platform(scene,'Trajectory',trajectory,'Dimensions', ...
    struct('Length',35,'Width',15,'Height',5.5,'OriginOffset',[0 0 -10]));

trajPlotter = trajectoryPlotter(tp,'DisplayName','Trajectory','Color','k','LineWidth',1.5);
plotTrajectory(trajPlotter,{trajectory.Waypoints})
targetPlotter = platformPlotter(tp,'DisplayName','Target', ...
    'Marker','s','MarkerEdgeColor','g','MarkerSize',2);
plotPlatform(targetPlotter,target.Position, ...
    target.Dimensions,quaternion(target.Orientation,'rotvecd'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Target 1
waypoints1 = [100 -100 10; 100 100 80];
trajectory1 = waypointTrajectory(waypoints1, [0 simulationDuration]);
target1 = platform(scene, 'Trajectory', trajectory1, 'Dimensions', ...
    struct('Length', 35, 'Width', 15, 'Height', 5.5, 'OriginOffset', [0 0 0]));
targetPlotter1 = platformPlotter(tp, 'DisplayName', 'Target 1', ...
    'Marker', 's', 'MarkerEdgeColor', 'g', 'MarkerSize', 2);
plotPlatform(targetPlotter1, target1.Position, ...
    target1.Dimensions, quaternion(target1.Orientation, 'rotvecd'));

% Target 2
waypoints2 = [-50 50 10; -50 -50 80];
trajectory2 = waypointTrajectory(waypoints2, [0 simulationDuration]);
target2 = platform(scene, 'Trajectory', trajectory2, 'Dimensions', ...
    struct('Length', 20, 'Width', 10, 'Height', 3, 'OriginOffset', [0 0 0]));
targetPlotter2 = platformPlotter(tp, 'DisplayName', 'Target 2', ...
    'Marker', 's', 'MarkerEdgeColor', 'r', 'MarkerSize', 2);
plotPlatform(targetPlotter2, target2.Position, ...
    target2.Dimensions, quaternion(target2.Orientation, 'rotvecd'));

% Target 3
waypoints3 = [-100 -100 10; -100 100 80];
trajectory3 = waypointTrajectory(waypoints3, [0 simulationDuration]);
target3 = platform(scene, 'Trajectory', trajectory3, 'Dimensions', ...
    struct('Length', 30, 'Width', 12, 'Height', 4, 'OriginOffset', [0 0 0]));
targetPlotter3 = platformPlotter(tp, 'DisplayName', 'Target 3', ...
    'Marker', 's', 'MarkerEdgeColor', 'b', 'MarkerSize', 2);
plotPlatform(targetPlotter3, target3.Position, ...
    target3.Dimensions, quaternion(target3.Orientation, 'rotvecd'));

% Target 4
waypoints4 = [-150 -50 10; 150 50 80];
trajectory4 = waypointTrajectory(waypoints4, [0 simulationDuration]);
target4 = platform(scene, 'Trajectory', trajectory4, 'Dimensions', ...
    struct('Length', 25, 'Width', 8, 'Height', 3.5, 'OriginOffset', [0 0 0]));
targetPlotter4 = platformPlotter(tp, 'DisplayName', 'Target 4', ...
    'Marker', 's', 'MarkerEdgeColor', 'm', 'MarkerSize', 2);
plotPlatform(targetPlotter4, target4.Position, ...
    target4.Dimensions, quaternion(target4.Orientation, 'rotvecd'));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on
plot3(tp.Parent,0,0,0,'Color','k','Marker','o','MarkerSize',4)


%% 


% Tower 1
tower1 = platform(scene, 'Position', [0, 50, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 2
tower2 = platform(scene, 'Position', [35.36, 35.36, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 3
tower3 = platform(scene, 'Position', [0, 50, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, 10]));

% Tower 4
tower4 = platform(scene, 'Position', [-35.36, 35.36, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 5
tower5 = platform(scene, 'Position', [-50, 0, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 6
tower6 = platform(scene, 'Position', [-35.36, -35.36, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 7
tower7 = platform(scene, 'Position', [0, -50, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Tower 8
tower8 = platform(scene, 'Position', [35.36, -35.36, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 30, 'OriginOffset', [0, 0, -10]));

% Display the towers using platform plotters
towerPlotter1 = platformPlotter(tp, 'DisplayName', 'Tower 1', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter1, tower1.Position, tower1.Dimensions, quaternion(tower1.Orientation, 'rotvecd'));

towerPlotter2 = platformPlotter(tp, 'DisplayName', 'Tower 2', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter2, tower2.Position, tower2.Dimensions, quaternion(tower2.Orientation, 'rotvecd'));

towerPlotter3 = platformPlotter(tp, 'DisplayName', 'Tower 3', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter3, tower3.Position, tower3.Dimensions, quaternion(tower3.Orientation, 'rotvecd'));

towerPlotter4 = platformPlotter(tp, 'DisplayName', 'Tower 4', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter4, tower4.Position, tower4.Dimensions, quaternion(tower4.Orientation, 'rotvecd'));

towerPlotter5 = platformPlotter(tp, 'DisplayName', 'Tower 5', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter5, tower5.Position, tower5.Dimensions, quaternion(tower5.Orientation, 'rotvecd'));

towerPlotter6 = platformPlotter(tp, 'DisplayName', 'Tower 6', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter6, tower6.Position, tower6.Dimensions, quaternion(tower6.Orientation, 'rotvecd'));

towerPlotter7 = platformPlotter(tp, 'DisplayName', 'Tower 7', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter7, tower7.Position, tower7.Dimensions, quaternion(tower7.Orientation, 'rotvecd'));

towerPlotter8 = platformPlotter(tp, 'DisplayName', 'Tower 8', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter8, tower8.Position, tower8.Dimensions, quaternion(tower8.Orientation, 'rotvecd'));


radar1 = radarDataGenerator(1,'DetectionMode','Monostatic', ...
    'UpdateRate',5, ...
    'MountingLocation',[0, 0, 30], ...
    'FieldOfView',[45,30],...
    'MechanicalAzimuthLimits',[0 45], ...
    'MechanicalElevationLimits',[ 0 0], ...
    'HasElevation',true, ...
    'RangeResolution',100, ...
    'AzimuthResolution',20, ...
    'ElevationResolution',20);
tower1.Sensors = radar1;

radarPlotter1 = coveragePlotter(tp,'Color','b','DisplayName','radar1 beam');


radar2= radarDataGenerator(2,'DetectionMode','Monostatic', ...
    'UpdateRate',5, ...
    'MountingLocation',[0, 0, 30], ...
    'FieldOfView',[45, 30],...
    'MechanicalAzimuthLimits',[45 90], ...
    'MechanicalElevationLimits',[0 0], ...
    'HasElevation',true, ...
    'RangeResolution',100, ...
    'AzimuthResolution',20, ...
    'ElevationResolution',20);
tower2.Sensors = radar2;

radarPlotter = coveragePlotter(tp,'Color','b','DisplayName','radar2 beam');


radar3= radarDataGenerator(3,'DetectionMode','Monostatic', ...
    'UpdateRate',5, ...
    'MountingLocation',[0, 0, 30], ...
    'FieldOfView',[45, 30],...
    'MechanicalAzimuthLimits',[90 135], ...
    'MechanicalElevationLimits',[0 0], ...
    'HasElevation',true, ...
    'RangeResolution',100, ...
    'AzimuthResolution',20, ...
    'ElevationResolution',20);
tower3.Sensors = radar3;

radarPlotter = coveragePlotter(tp,'Color','b','DisplayName','radar3 beam');



radar4= radarDataGenerator(4,'DetectionMode','Monostatic', ...
    'UpdateRate',5, ...
    'MountingLocation',[0, 0, 30], ...
    'FieldOfView',[45, 30],...
    'MechanicalAzimuthLimits',[135 180], ...
    'MechanicalElevationLimits',[0 0], ...
    'HasElevation',true, ...
    'RangeResolution',100, ...
    'AzimuthResolution',20, ...
    'ElevationResolution',20);
tower4.Sensors = radar4;

radarPlotter = coveragePlotter(tp,'Color','b','DisplayName','radar4 beam');

% Radar 5 for Tower 5
radar5 = radarDataGenerator(5, 'DetectionMode', 'Monostatic', ...
    'UpdateRate', 5, ...
    'MountingLocation', [0, 0, 30], ...
    'FieldOfView', [45, 30], ...
    'MechanicalAzimuthLimits', [180 225], ... % Customize azimuth limits as needed
    'MechanicalElevationLimits', [0 0], ...
    'HasElevation', true, ...
    'RangeResolution', 100, ...
    'AzimuthResolution', 10, ...
    'ElevationResolution', 10);
tower5.Sensors = radar5;


% Display Radar 5 coverage
radarPlotter = coveragePlotter(tp, 'Color', 'y', 'DisplayName', 'Radar5 Beam');

% Radar 6 for Tower 6
radar6 = radarDataGenerator(6, 'DetectionMode', 'Monostatic', ...
    'UpdateRate', 5, ...
    'MountingLocation', [0, 0, 30], ...
    'FieldOfView', [45, 30], ...
    'MechanicalAzimuthLimits', [225 270], ... % Customize azimuth limits as needed
    'MechanicalElevationLimits', [0 0], ...
    'HasElevation', true, ...
    'RangeResolution', 100, ...
    'AzimuthResolution', 20, ...
    'ElevationResolution', 20);
tower6.Sensors = radar6;

% Display Radar 6 coverage
radarPlotter = coveragePlotter(tp, 'Color', 'g', 'DisplayName', 'Radar6 Beam');

% Radar 7 for Tower 7
radar7 = radarDataGenerator(7, 'DetectionMode', 'Monostatic', ...
    'UpdateRate', 5, ...
    'MountingLocation', [0, 0, 30], ...
    'FieldOfView', [45, 30], ...
    'MechanicalAzimuthLimits', [270 315], ... % Customize azimuth limits as needed
    'MechanicalElevationLimits', [0 0], ...
    'HasElevation', true, ...
    'RangeResolution', 100, ...
    'AzimuthResolution', 20, ...
    'ElevationResolution', 20);
tower7.Sensors = radar7;

% Display Radar 7 coverage
radarPlotter7 = coveragePlotter(tp, 'Color', 'c', 'DisplayName', 'Radar7 Beam');

% Radar 6 for Tower 6
radar8 = radarDataGenerator(8, 'DetectionMode', 'Monostatic', ...
    'UpdateRate', 5, ...
    'MountingLocation', [0, 0, 30], ...
    'FieldOfView', [45, 30], ...
    'MechanicalAzimuthLimits', [315 360], ... % Customize azimuth limits as needed
    'MechanicalElevationLimits', [0 0], ...
    'HasElevation', true, ...
    'RangeResolution', 100, ...
    'AzimuthResolution', 20, ...
    'ElevationResolution', 20);
tower8.Sensors = radar8;

% Display Radar 8 coverage
radarPlotter8 = coveragePlotter(tp, 'Color', 'k', 'DisplayName', 'Radar8 Beam');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create and display the towers in a circular pattern
% Tower 1
tower11 = platform(scene, 'Position', [0, 150, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 2
tower12 = platform(scene, 'Position', [106.066, 106.066, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 3
tower13 = platform(scene, 'Position', [150, 0, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, 10]));

% Tower 4
tower14 = platform(scene, 'Position', [106.066, -106.066, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 5
tower15 = platform(scene, 'Position', [0, -150, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 6
tower16 = platform(scene, 'Position', [-106.066, -106.066, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 7
tower17 = platform(scene, 'Position', [-150, 0, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 230, 'OriginOffset', [0, 0, -10]));

% Tower 8
tower18 = platform(scene, 'Position', [-106.066, 106.066, 0], 'Dimensions', ...
    struct('Length', 5, 'Width', 5, 'Height', 220, 'OriginOffset', [0, 0, -10]));

towerPlotter11 = platformPlotter(tp, 'DisplayName', 'Tower 11', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter11, tower11.Position, tower11.Dimensions, quaternion(tower11.Orientation, 'rotvecd'));

towerPlotter12 = platformPlotter(tp, 'DisplayName', 'Tower 12', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter12, tower12.Position, tower12.Dimensions, quaternion(tower12.Orientation, 'rotvecd'));

towerPlotter13 = platformPlotter(tp, 'DisplayName', 'Tower 13', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter13, tower13.Position, tower13.Dimensions, quaternion(tower13.Orientation, 'rotvecd'));

towerPlotter14 = platformPlotter(tp, 'DisplayName', 'Tower 14', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter14, tower14.Position, tower14.Dimensions, quaternion(tower14.Orientation, 'rotvecd'));

towerPlotter15 = platformPlotter(tp, 'DisplayName', 'Tower 15', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter15, tower15.Position, tower15.Dimensions, quaternion(tower15.Orientation, 'rotvecd'));

towerPlotter16 = platformPlotter(tp, 'DisplayName', 'Tower 16', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter16, tower16.Position, tower16.Dimensions, quaternion(tower16.Orientation, 'rotvecd'));

towerPlotter17 = platformPlotter(tp, 'DisplayName', 'Tower 17', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter17, tower17.Position, tower17.Dimensions, quaternion(tower17.Orientation, 'rotvecd'));

towerPlotter18 = platformPlotter(tp, 'DisplayName', 'Tower 18', 'Marker', 's', 'MarkerSize', 2);
plotPlatform(towerPlotter18, tower18.Position, tower18.Dimensions, quaternion(tower18.Orientation, 'rotvecd'));


detPlotter1 = detectionPlotter(tp,'DisplayName','Detection','MarkerFaceColor','r','MarkerSize',10);
detPlotter8 = detectionPlotter(tp,'DisplayName','Detection','MarkerFaceColor','r','MarkerSize',10);

% Display Radar 8 coverage

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rng(2019) % for repeatable results
while advance(scene)
% Plot target.
plotPlatform(targetPlotter,target.Position, ...
target.Dimensions,quaternion(target.Orientation,'rotvecd'))

 
    plotCoverage(radarPlotter8, coverageConfig(scene))

 poseInTower1 = targetPoses(tower1);
 [detections1, numDets1] = radar1(poseInTower1,scene.SimulationTime);

poseInTower2 = targetPoses(tower2);
 [detections2, numDets2] = radar2(poseInTower2,scene.SimulationTime);

poseInTower3 = targetPoses(tower3);
 [detections3, numDets3] = radar3(poseInTower3,scene.SimulationTime);

poseInTower4 = targetPoses(tower4);
 [detections4, numDets4] = radar4(poseInTower4,scene.SimulationTime);

poseInTower5 = targetPoses(tower5);
 [detections5, numDets5] = radar5(poseInTower5,scene.SimulationTime);

poseInTower6 = targetPoses(tower6);
 [detections6, numDets6] = radar6(poseInTower6,scene.SimulationTime);

 poseInTower7 = targetPoses(tower7);
 [detections7, numDets7] = radar7(poseInTower7,scene.SimulationTime);

 poseInTower8 = targetPoses(tower8);
    [detections8, numDets8] = radar8(poseInTower8, scene.SimulationTime);

    % Process and plot detections for tower 8
   detPos1 = zeros(numDets1, 3);
    detNoise1 = zeros(3, 3, numDets1);

    for i = 1:numDets1
        detPos1(i, :) = tower1.Trajectory.Position + detections1{i}.Measurement';
        detNoise1(:, :, i) = detections1{i}.MeasurementNoise;
    end

    if ~isempty(detPos1)
        plotDetection(detPlotter1, detPos1, detNoise1)
    end

    % Process and plot detections for tower 8
    detPos8 = zeros(numDets8, 3);
    detNoise8 = zeros(3, 3, numDets8);

    for i = 1:numDets8
        detPos8(i, :) = tower8.Trajectory.Position + detections8{i}.Measurement';
        detNoise8(:, :, i) = detections8{i}.MeasurementNoise;
    end

    if ~isempty(detPos8)
        plotDetection(detPlotter8, detPos8, detNoise8)
    end
end