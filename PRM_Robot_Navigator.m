load('Maps.mat')

mapWaypoints = [5 36; 23 36; 35 36; 46 36; 5 28; 5 15; 24 16; 47 28; 45 16; 4 3];

simMap = binaryOccupancyMap(complexMap);
lidarMap = imageToOccupancy(complexMap,5);

loopPts = mapWaypoints([10 6 5 8 9 10],:);

% Lidar Parameters
lidarMaxRange = 20;
setAngles = -pi:pi/50:pi;
slamMaxRange = 15;
slamResolution = 10;

% Robot Parameters
trackWidth = 1.75;
initialOrientation = 0;
currentPose(3) = initialOrientation;

% Simulation Parameters
frameSize = trackWidth/0.8;
fps = 15;
goalRadius = 0.5;


% SLAM
slamObj = lidarSLAM(slamResolution, slamMaxRange);

% Robot Controller
robot = bicycleKinematics("MaxSteeringAngle",pi/4);

% PRM
mapInflated = copy(simMap);
inflate(mapInflated, trackWidth/2);
prm = mobileRobotPRM(mapInflated, 2000);
prm.ConnectionDistance = 10;

% Add obstacle


%% Driving
for i = 2:size(loopPts, 1)
    % Set Waypoint
    startLocation = loopPts(i-1,:);
    endLocation = loopPts(i,:);
    % PRM
    path = findpath(prm,startLocation,endLocation);
    
    while isempty(path)
        % No feasible path found yet, increase the number of nodes
        prm.NumNodes = prm.NumNodes + 10;

        % Use the |update| function to re-create the PRM roadmap with the changed
        % attribute
        update(prm);

        % Search for a feasible path with the updated PRM
        path = findpath(prm, startLocation, endLocation);
    end
    
    figure(1)
    show(prm)
    
    
    % Robot Controller
    controller = controllerPurePursuit('Waypoints',path,'DesiredLinearVelocity',2,'LookaheadDistance', 2);
    robotInitialLocation = path(1,:);
    robotGoal = path(end,:);

    currentPose = [robotInitialLocation currentPose(3)]';
    calcPose = currentPose;

    distanceToGoal = norm(robotInitialLocation - robotGoal);

    % Initialize the figure
    figure(2)
    xlim([0 52])
    ylim([0 41])

    while( distanceToGoal > goalRadius )
        angles = setAngles+currentPose(3);
        [scan,lidarPts] = simLidar(setAngles,simMap,currentPose,lidarMaxRange);
        
        
        %addScan(slamObj, scan);
        insertRay(lidarMap,currentPose,scan,lidarMaxRange);
        
        scanOld = scan;
        calcDeltaPose = matchScans(scan, scanOld)';
        calcPose = calcPose + calcDeltaPose;

        %% Robot Controller
        
        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(currentPose);

        % Get the robot's velocity using controller inputs
        vel = derivative(robot, currentPose, [v omega]);

        % Update the current pose
        currentPose = currentPose + vel/fps;

        % Re-compute the distance to the goa
        distanceToGoal = norm(currentPose(1:2) - robotGoal(:));


        plotRobot(simMap,path,frameSize,currentPose,calcPose,lidarPts,lidarMaxRange,angles);
        
        figure(3)
        show(lidarMap)
    end
end    


