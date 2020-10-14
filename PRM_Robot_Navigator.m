load('Maps.mat')

mapWaypoints = [5 36; 23 36; 35 36; 46 36; 5 28; 5 15; 24 16; 47 28; 45 16; 4 3];

map = occupancyMap(complexMap);

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


%% Driving
for i = 2:size(loopPts, 1)
    % Set Waypoint
    startLocation = loopPts(i-1,:);
    endLocation = loopPts(i,:);
    
    % PRM
    path = pathFinder(map,trackWidth,startLocation, endLocation);

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
        [scan,lidarPts] = simLidar(setAngles,map,currentPose,lidarMaxRange);
        
        
        %addScan(slamObj, scan);
        insertRay(map,currentPose,scan,lidarMaxRange);
        
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


        plotRobot(map,path,frameSize,currentPose,calcPose,lidarPts,lidarMaxRange,angles);
        
        figure(3)
        show(map);
    end
end    


