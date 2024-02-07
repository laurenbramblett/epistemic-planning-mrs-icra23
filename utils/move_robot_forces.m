function force = move_robot_forces(robot,goal,obsInRange)
    dt    = robot.dt;
    pose  = robot.pose'; 
    max_v = robot.vel;
    obsRho = 5;
    obsSpringGain = 10;
    goalSpringGain = 1;
    
    % ------------------- %
    %% Obs Force
    % ------------------- %
    obs       = obsInRange;
    if ~isempty(obs)
        obsDist = pdist2(pose(1:2),obs)';
    else
        obsDist = [];
    end
    c_obs = obs(obsDist<obsRho,:); obsDist = obsDist(obsDist<obsRho,:);
    if ~isempty(c_obs)
        [minVal,minIdx] = min(obsDist);
        dirObs = c_obs(minIdx,:)-pose(1:2);
        if minVal>1.5
            obsSpring = -(dirObs)./minVal*(1/1.5-1/minVal)*(1/minVal.^2);
        else
            obsSpring = -sign(dirObs).*[1000,1000];
        end
    else
        obsSpring = [0,0]; 
    end

    % ------------------- %
    %% Goal Force
    % ------------------- %
    if isempty(goal)
      goal = robot.depot(1:2)';
    end
    if size(pose(1:2),1)~=size(goal,1)
        s = 1;
    end
    goalDist = pdist2(pose(1:2),goal);

    if goalDist>0
        goalNorm = (goal-pose(1:2))./goalDist;
    else
        goalNorm = [0,0];
    end
    
    goalSpring = goalDist*goalNorm;
    if sqrt(sum(goalSpring.^2))>max_v
        goalSpring = goalSpring*max_v/sqrt(sum(goalSpring.^2));
    end    
    
    % ------------------- %
    %% Combined Force
    % ------------------- %
    v = goalSpringGain*goalSpring + obsSpringGain*obsSpring;

    if sqrt(sum(v.^2))/dt > max_v
        v = v*max_v/sqrt(sum(v.^2));
    end 
    
    if norm(v) < 0.1
        v = [0,0];
    end
    force = v;
end