function [forceX,forceY] = apfModel_robot(robot,map,obsInRange,goal)
    maxRange = robot.maxRange;
    gridPoints = map.gridPoints;
    otherBots = robot.guessLocs;
    assignedFrontier = gridPoints(frontier,:);
    assignedFrontier = assignedFrontier(frontIdx(:,robot.ID),:);
    frontierWeight = norm(assignedFrontier-robot.pose(1:2)');
    
    frontierX = goal(1)-robot.pose(1);
    frontierY = goal(2)-robot.pose(2);
    if isempty(obsInRange)
        obsForceX = 0; obsForceY = 0;
    else
        closeObs = obsInRange(norm(obsInRange-robot.pose(1:2)')<maxRange*1.9/4,:);
        if sum(closeObs)<1
            obsForceX = 0; obsForceY = 0;
        else
            obsWeight = norm(closeObs-robot.pose(1:2)');
            obsX = closeObs(:,1)-robot.pose(1);
            obsY = closeObs(:,2)-robot.pose(2);
            obsForceX = sum(obsX./obsWeight.^3)/length(obsWeight);
            obsForceY = sum(obsY./obsWeight.^3)/length(obsWeight);
        end
    end

    frontierForceX = sum(frontierX./frontierWeight.^3)/length(frontierWeight);
    frontierForceY = sum(frontierY./frontierWeight.^3)/length(frontierWeight);
    if isnan(frontierForceX)
        frontierForceX = 0; frontierForceY = 0;
    end
    
    %OtherRobotForces
    agents = size(otherBots,1);
    otherIdx = setdiff(1:agents,robot.ID);
    distOther = norm(robot.pose(1:2)'-otherBots(otherIdx,1:2));
    whichForces = distOther<maxRange*100;
    multiplier = ones(1,agents-1)*robot.coeffs.otherBots;
    if sum(whichForces)>0
        distOtherX = otherBots(otherIdx(whichForces),1)-robot.pose(1);
        distOtherY = otherBots(otherIdx(whichForces),2)-robot.pose(2);
        divideBy = sum(whichForces);
%         otherForceX = sum(distOtherX'./distOther.^3)./(divideBy);
%         otherForceY = sum(distOtherY'./distOther.^3)./(divideBy);
        multiplier(distOther<2/2*maxRange) = 0;
        multiplier(distOther<1) = -1;
        otherForceX = sum(multiplier.*distOtherX'./distOther.^3)./(divideBy);
        otherForceY = sum(multiplier.*distOtherY'./distOther.^3)./(divideBy);
    else
    	otherForceX = 0;
        otherForceY = 0;
    end
    
    %Goal force
    if ~isempty(robot.goalQueue) && robot.goalState == "task"
        goal = robot.goalQueue(1,:);
        goalForceX = goal(1)-pose(1);
        goalForceY = goal(2)-pose(2);
    else
        goalForceX = 0; goalForceY = 0;
    end
    
    forceX = robot.coeffs.frontier*frontierForceX + robot.coeffs.obs*obsForceX + otherForceX + robot.coeffs.goal*goalForceX;
    forceY = robot.coeffs.frontier*frontierForceY + robot.coeffs.obs*obsForceY + otherForceY + robot.coeffs.goal*goalForceY;

end