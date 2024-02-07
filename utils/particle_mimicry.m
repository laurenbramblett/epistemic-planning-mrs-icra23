zCellsOther = zeros(1000,3); indCells = 1;
%Update occupancy map for common belief
for r = 1:numAgents
    [zCellsR,~,~,~] = lidarSim(robots(k).particles(r,1),map);
    zCellsOther(indCells:(indCells+size(zCellsR,1)-1),:) = zCellsR;
    indCells = indCells + size(zCellsR,1); 
end
zCells = zCellsOther(1:indCells-1,:);
Mp = updateOccupancyMap(zCells,robots(k).particles(k,1).M0(:),map);

%Update goals and propagate particles
for r = 1:numAgents
    robots(k).particles(r,1).M0 = reshape(Mp,map.size);
    particleFront = frontierTest(robots(k).particles(r,1),map);
    idxF_particle = partitionFunc(particleFront,robots(k),map);
    frontier(:,k) = particleFront;
    attractiveFront{k} = idxF_particle;
    idxF = idxF_particle;

    for p = 1:numParticles
        robots(k).particles(r,p).coeffs = robots(k).coeffs;
        if isempty(idxF)
            robots(k).particles(r,p).coeffs.otherBots = 1;
        end
        robots(k).particles(r,p).M0 = reshape(Mp,map.size);
        % [~,~,obsInRangeP,~] = lidarSim(robots(k).particles(r,p),map);
        [pforceX,pforceY] = move_particle_forces(robots(k).particles(r,p),map,...
                                   frontier(:,k),idxF);
        if robots(k).particles(r,p).state == "task"
            goal = robots(k).particles(r,p).goalQueue(1,:);
            check = norm(robots(k).particles(r,p).pose(1:2)-goal)<completeRange;
            if check
                robots(k).particles(r,p).goalQueue(1,:) = [];
            end
            if isempty(robots(k).particles(r,p).goalQueue)
                robots(k).particles(r,p).FC = FC;
            end
        end

        robots(k).particles(r,p) = propagateParticles(robots(k).particles(r,p),pforceX,pforceY);   
    end
end
best_guess