function frontier = frontierTest(robot,map)
    pose = robot.pose;
    gridPoints = map.gridPoints;
    occMap = reshape(robot.M0,map.size);
    mapSize = map.size;
    frontier = zeros(length(gridPoints),1);
    for i = 1:length(gridPoints)
        M = zeros(mapSize);
        if occMap(gridPoints(i,2),gridPoints(i,1))<0.15 && ~isequal(gridPoints(i,:),pose(1:2))
            M(gridPoints(i,2),gridPoints(i,1)) = 1; % location
            vals = occMap(conv2(M,[0,1,0;1,0,1;0,1,0],'same')>0);
            frontier(i) = any(vals<0.65 & vals>0.45) && any(vals<0.15);
        end
    end
    frontier = logical(frontier);
end