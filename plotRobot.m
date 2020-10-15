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
