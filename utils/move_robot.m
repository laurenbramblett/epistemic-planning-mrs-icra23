if connected
    frontier(:,k) = frontierTest(robots(k),map);
    idxF = partitionFunc(frontier(:,k),robots(k),map);
    attractiveFront{k} = idxF;
    [forceX,forceY] = move_particle_forces(robots(k),map,frontier(:,k),idxF);
    velAtt = robots(k).vel; 
    angle = wrapToPi(atan2(forceY,forceX)-robots(k).pose(3));
%     angle =
    kp = 0.3; angle = sign(angle)*min(abs(angle),pi/4);
    %calculate attraction to belief
    task_check
else
%             particle_mimicrySim
    belief = robots(k).particles(k,robots(k).particleFollow).pose;
    beliefx = belief(1);
    beliefy = belief(2);
    goal = [beliefx,beliefy];
    task_check
    
    if ~isempty(obsInRange{k}) && size(obsInRange{k},1)>obNum
        [~,cIdx] = sort(pdist2(obsInRange{k},robots(k).pose(1:2)'));
        obsOpt = obsInRange{k}(cIdx(1:obNum),:);
    else
        nObs = size(obsInRange{k},1);
        obsOpt = [obsInRange{k}; ones(obNum-nObs,2)*-5];
    end
    obsOpt = reshape(obsOpt',[],1);
    x0 = [robots(k).pose; robots(k).vel+epsilon; goal'; obsOpt];
    [uk,diag] = controller{x0};
    velAtt = norm(uk);
    % uk = move_robot_forces(robots(k),goal,obsInRange{k});
    % if sqrt(sum(uk.^2))>robots(k).vel
    %     uk = uk*robots(k).vel/sqrt(sum(uk.^2));
    % end    
    % velAtt = norm(uk) + epsilon;
    angle = wrapToPi(atan2(uk(2),uk(1))-robots(k).pose(3));
end

% robots(k).M0 = robots(k).M1;
%Self movement

% angle = sign(angle)*min(abs(angle),maxTurn);
% velAtt = velAtt*abs(cos(angle));
robots(k).pose(3) = wrapToPi(robots(k).pose(3) + angle);
robots(k).pose(1) = robots(k).pose(1) + (velAtt)*cos(robots(k).pose(3))*dt;
robots(k).pose(2) = robots(k).pose(2) + (velAtt)*sin(robots(k).pose(3))*dt;
if any(isnan(robots(k).pose))
    s=1;
end
    