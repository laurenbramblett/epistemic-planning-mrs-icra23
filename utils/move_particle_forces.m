function [forceX,forceY] = move_particle_forces(robot,map,frontier,frontIdx)
    gridPoints = map.gridPoints;
    otherBots = robot.guessLocs;
    obsRho = 3;
    assignedFrontier = gridPoints(frontier,:);
    assignedFrontier = assignedFrontier(frontIdx(:,robot.ID),:);
    headingWeight = abs(wrapToPi(atan2(assignedFrontier(:,2)-robot.pose(2),assignedFrontier(:,1)-robot.pose(1))-robot.pose(3)));
    frontierWeight = vecnorm(assignedFrontier-robot.pose(1:2)',2,2) + 1*headingWeight;
    
    frontierX = assignedFrontier(:,1)-robot.pose(1);
    frontierY = assignedFrontier(:,2)-robot.pose(2);
    frontierForceX = sum(frontierX./frontierWeight.^3);
    frontierForceY = sum(frontierY./frontierWeight.^3);
    if isnan(frontierForceX)
        frontierForceX = 0; frontierForceY = 0;
    end
    frontierSpring = [frontierForceX,frontierForceY];
    frontierSpring = frontierSpring/(sqrt(sum(frontierSpring.^2))+1e-4);  
    frontierForceX = frontierSpring(1); frontierForceY = frontierSpring(2);

    % ------------------- %
    %% Obs Force
    % ------------------- %
    [r_o,c_o] = find(robot.M0>0.5);
    obs = [c_o,r_o];
    if ~isempty(obs)
        obsDist = pdist2(robot.pose(1:2)',obs)';
    else
        obsDist = [];
    end
    c_obs = obs(obsDist<obsRho,:); obsDist = obsDist(obsDist<obsRho,:);
    if ~isempty(c_obs)
        [minVal,minIdx] = min(obsDist);
        dirObs = c_obs(minIdx,:)-robot.pose(1:2)';
        if minVal>1.2
            obsSpring = -(dirObs)./minVal*(1/1.5-1/minVal)*(1/minVal.^2);
        else
            obsSpring = -sign(dirObs).*[1000,1000];
        end
        obsSpring = obsSpring/(sqrt(sum(obsSpring.^2))+1e-4);
    else
        obsSpring = [0,0]; 
    end
    
    obsForceX = obsSpring(1); obsForceY = obsSpring(2);
    
    %OtherRobotForces
    agents = size(otherBots,2);
    otherIdx = setdiff(1:agents,robot.ID);
    otherWeight = vecnorm(otherBots(1:2,otherIdx)'-robot.pose(1:2)',2,2);

    distOtherX = otherBots(1,otherIdx)'-robot.pose(1);
    distOtherY = otherBots(2,otherIdx)'-robot.pose(2);
    otherForceX = sum(distOtherX./otherWeight.^3);
    otherForceY = sum(distOtherY./otherWeight.^3);
    otherSpring = [otherForceX,otherForceY];
    otherSpring = otherSpring/(sqrt(sum(otherSpring.^2))+1e-4);
    otherForceX = otherSpring(1); otherForceY = otherSpring(2);
    
    %Goal force
    if ~isempty(robot.goalQueue) && robot.goalState == "task"
        goal = robot.goalQueue(1,:);
        goalForceX = goal(1)-pose(1);
        goalForceY = goal(2)-pose(2);
    else
        goalForceX = 0; goalForceY = 0;
    end
    goalSpring = [goalForceX,goalForceY];
    if sqrt(sum(goalSpring.^2))>robot.vel
        goalSpring = goalSpring*robot.vel/(sqrt(sum(goalSpring.^2))+1e-4);
    end    
    goalForceX = goalSpring(1); goalForceY = goalSpring(2);
    
    forceX = robot.coeffs.frontier*frontierForceX + robot.coeffs.obs*obsForceX + robot.coeffs.otherBots*otherForceX + robot.coeffs.goal*goalForceX;
    forceY = robot.coeffs.frontier*frontierForceY + robot.coeffs.obs*obsForceY + robot.coeffs.otherBots*otherForceY + robot.coeffs.goal*goalForceY;

end