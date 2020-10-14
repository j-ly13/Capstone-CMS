load('Maps.mat')

mapWaypoints = [5 36; 23 36; 35 36; 46 36; 5 28; 5 15; 24 16; 47 28; 45 16; 4 3];

map = binaryOccupancyMap(complexMap);

loopPts = mapWaypoints([10 6 5 8 9 10],:);

% Lidar Parameters
lidarMaxRange = 20;
setAngles = -pi:pi/200:pi;
slamMaxRange = 15;
slamResolution = 5;

% Robot Parameters
trackWidth = 1.75;
initialOrientation = 0;
currentPose(3) = initialOrientation;

% Simulation Parameters
frameSize = trackWidth/0.8;
fps = 15;
goalRadius = 0.5;

% PRM
mapInflated = copy(map);
inflate(mapInflated, trackWidth/2);
prm = robotics.PRM(mapInflated);
prm.ConnectionDistance = 10;

% SLAM
slamObj = lidarSLAM(slamResolution, slamMaxRange);

mcl = monteCarloLocalization('InitialPose',currentPose,'UseLidarScan',true);

% Robot Controller
robot = bicycleKinematics("MaxSteeringAngle",pi/4);


%% Driving
for i = 2:size(loopPts, 1)
    % Set Waypoint
    startLocation = loopPts(i-1,:);
    endLocation = loopPts(i,:);
    
    % PRM
    prm.NumNodes = 1000;

    path = findpath(prm, startLocation, endLocation);

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
    show(prm);

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
        % Simulate Lidar
        angles = setAngles+currentPose(3);
        lidarPts = rayIntersection(map,currentPose,setAngles,lidarMaxRange);

        % Create Lidar Scan Data
        lidarVectors = lidarPts-repmat(transpose(currentPose([1 2])),size(lidarPts,1),1 );
        lidarRanges = hypot(lidarVectors(:,1),lidarVectors(:,2));
        %scanOld = scan;
        scan = lidarScan(lidarRanges,transpose(setAngles) );

        addScan(slamObj, scan);
        %calcDeltaPose = matchScans(scan, scanOld)';
        %calcPose = calcPose + calcDeltaPose;

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
        show(slamObj);
    end
end    


function plotRobot(map,path,frameSize,currentPose,calcPose,lidarPts,lidarMaxRange,angles)
    % Update the plot
    figure(2)
    hold off
    show(map);
    hold all

    % Lidar Plots
    plot(lidarPts(:,1),lidarPts(:,2),'*r') % Intersection points
    for i = 1:size(lidarPts,1)
        if ~isnan(lidarPts(i))
            plot([currentPose(1),lidarPts(i,1)],[currentPose(2),lidarPts(i,2)],'-b') % Plot intersecting rays
        else
            plot([currentPose(1),currentPose(1)+lidarMaxRange*cos(angles(i))],[currentPose(2),currentPose(2)+lidarMaxRange*sin(angles(i))],'-b') % No intersection ray
        end
    end
    %Plot path each instance so that it stays persistent while robot mesh moves
    plot(path(:,1), path(:,2),"k--d")

    % Plot the path of the robot as a set of transforms
    plotTrVec = [currentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 currentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshColor', 'red', 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);

    % Plot the path of the calculated movement
    %plotTrVec = [calcPose(1:2); 0];
    %plotRot = axang2quat([0 0 1 calcPose(3)]);
    %plotTransforms(plotTrVec', plotRot, 'MeshColor', 'green', 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    
    drawnow;
end
