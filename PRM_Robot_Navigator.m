load('Maps.mat')

mapWaypoints = [5 36; 23 36; 35 36; 46 36; 5 28; 5 15; 24 16; 47 28; 45 16; 4 3];

simMap = binaryOccupancyMap(complexMap);
lidarMap = imageToOccupancy(complexMap,5);

loopPts = mapWaypoints([10 6 5 8 9 10],:);

% Lidar Parameters
lidarMaxRange = 20;
setAngles = -pi:pi/200:pi;
slamMaxRange = 15;
slamResolution = 10;

% Robot Parameters
trackWidth = 1.75;
initialOrientation = pi/2;
currentPose(3) = initialOrientation;

% Simulation Parameters
frameSize = trackWidth/0.8;
fps = 10;
goalRadius = 0.75;

% Robot Controller
robot = bicycleKinematics("MaxSteeringAngle",pi/4);

% PRM
mapInflated = copy(simMap);
inflate(mapInflated, trackWidth/2);
prm = mobileRobotPRM(mapInflated, 500);
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
    controller = controllerPurePursuit('Waypoints',path,'DesiredLinearVelocity',2,'LookaheadDistance', 2.5);
    robotGoal = path(end,:);

    initialPose = [path(1,:) currentPose(3)]';
    currentPose = initialPose;
    calcPose = currentPose;
    distanceToGoal = norm(calcPose(1:2) - robotGoal(:));
    
    [scan,lidarPts] = simLidar(setAngles,simMap,currentPose,lidarMaxRange);
    
    
    % SLAM
    slamObj = lidarSLAM(slamResolution, slamMaxRange);
    
    % Initialize the figure
    figure(2)
    xlim([0 52])
    ylim([0 41])

    while( distanceToGoal > goalRadius )
        angles = setAngles+currentPose(3);
        scanOld = scan;
        [scan,lidarPts] = simLidar(setAngles,simMap,currentPose,lidarMaxRange);
        
        addScan(slamObj, scan);
        insertRay(lidarMap,currentPose,scan,lidarMaxRange);
        
        [scans,poses] = scansAndPoses(slamObj);
        calcPose = ( poses( size(poses,1) ,:) )';
        calcPose(1:2) = (calcPose(1:2)'*[cos(-initialPose(3)) -sin(-initialPose(3)); sin(-initialPose(3)) cos(-initialPose(3))])'+initialPose(1:2);
        calcPose(3) = wrapToPi(calcPose(3) + initialPose(3));
        
        %% Robot Controller
        
        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = controller(calcPose);

        % Get the robot's velocity using controller inputs
        vel = derivative(robot, currentPose, [v omega]);

        % Update the current pose
        currentPose = currentPose + vel/fps;
        currentPose(3) = wrapToPi(currentPose(3));
        
        % Re-compute the distance to the goal
        distanceToGoal = norm(calcPose(1:2) - robotGoal(:));


        plotRobot(lidarMap,path,frameSize,currentPose,calcPose,lidarPts,lidarMaxRange,angles);
        
        %figure(3)
        %show(slamObj) % Show the lidarSLAM object. As more scans are
        %added, this will become more intensive
    end
end    


