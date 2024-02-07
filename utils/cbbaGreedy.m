function match = cbbaGreedy(frontier,robot,map)
    loc = robot.guessLocs;
    gridPoints = map.gridPoints;
    vel = robot.vel;
    dmat = pdist2(loc(1:2,:)',gridPoints(frontier,:))';
    cost = dmat./vel;
    agents = size(cost,2);
    frontNum = size(cost,1);
    match = zeros(frontNum,agents);

    for f = 1:frontNum
        addNum = cost(f,:);
        [~,idx] = min(addNum);
        if sum(match(:,idx))<=frontNum*0.9
            match(f,idx) = 1;
        else
            while sum(match(:,idx))>frontNum*0.9
                cost(f,idx) = cost(f,idx)+1e4;
                addNum = cost(f,:);
                [~,idx] = min(addNum);
            end
            match(f,idx) = 1;
        end
    end      
    match = logical(match);
end