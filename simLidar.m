function [scan,lidarPts] = simLidar(setAngles,map,currentPose,lidarMaxRange)
    % Simulate Lidar
    lidarPts = rayIntersection(map,currentPose,setAngles,lidarMaxRange);

    % Create Lidar Scan Data
    lidarVectors = lidarPts-repmat(transpose(currentPose([1 2])),size(lidarPts,1),1 );
    lidarRanges = hypot(lidarVectors(:,1),lidarVectors(:,2));
    scan = lidarScan(lidarRanges,transpose(setAngles) );
end