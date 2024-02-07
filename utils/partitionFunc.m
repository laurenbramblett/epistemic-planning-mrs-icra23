function match = partitionFunc(frontier,robot,map)
    loc = robot.guessLocs;
    gridPoints = map.gridPoints;
    vel = robot.vel;
    front_points = gridPoints(frontier,:);
    agents = size(loc,2);
    dmat = pdist2(loc(1:2,:)',front_points)';
    cost = dmat./vel;
    frontNum = size(cost,1);
    match = false(frontNum,agents);
    if sum(frontier)>0
        [~,partition] = min(dmat,[],2);
        for p = 1:frontNum
            match(p,partition(p)) = 1;
        end
        howMany = sum(match,1);
        for r = 1:agents
            if howMany(r)<1
                [~,idx] = min(cost(:,r));
                match(idx,:) = zeros(1,agents);
                match(idx,r) = true;
            end
        end
    end
end