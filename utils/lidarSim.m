function [zCells,rayPts,obsInRange,endPts] = lidarSim(robot,map)
    % Definitions and preallocations
    maxRange = robot.maxRange;
    angles = robot.angles;
    gridSize = map.size;
    obs = map.obs;
    pos = robot.pose;
    numAngles = length(angles);
    rayPts = zeros(maxRange * numAngles, 3); 
    endPts = zeros(maxRange * numAngles, 2);
    obsInRange = zeros(numAngles, 2); % Preallocating with a maximum possible size
    obsInRangeCount = 0;

    x1 = pos(1);
    y1 = pos(2);
    anglesH = pos(3) + angles;
    x2 = x1 + cos(anglesH) * maxRange;
    y2 = y1 + sin(anglesH) * maxRange;

    rpCount = 0;
    epCount = 0;

    % Main loop
    for i = 1:numAngles
        % Create a set of linearly spaced coordinates between (x1,y1) and (x2(i),y2(i))
        xu = linspace(x1, x2(i), maxRange);
        yu = linspace(y1, y2(i), maxRange);
        
        x = round(xu);
        y = round(yu);
        
        validIndices = (x >= 1 & x <= gridSize(1) & y >= 1 & y <= gridSize(2));
        xu = xu(validIndices);
        yu = yu(validIndices);
        x = x(validIndices);
        y = y(validIndices);

        % Check for obstacles
        obsLogic = any(all(bsxfun(@eq, reshape(obs, 1, 2, []), [x;y]'), 2), 3);
        obsLogic = any(all(bsxfun(@eq, permute([x;y]', [1 3 2]), permute(obs, [3 1 2])), 3), 2);
        
        firstObsIndex = find(obsLogic, 1, 'first');
        
        % Store results
        numPoints = numel(xu);
        rayPts(rpCount + 1:rpCount + numPoints, :) = [xu' yu' obsLogic];
        rpCount = rpCount + numPoints;

        if ~isempty(firstObsIndex)
            obsInRangeCount = obsInRangeCount + 1;
            obsInRange(obsInRangeCount, :) = [x(firstObsIndex), y(firstObsIndex)];
            endPts(epCount + 1, :) = [xu(firstObsIndex), yu(firstObsIndex)];
            epCount = epCount + 1;
        end

        endPts(epCount + 1, :) = [xu(end), yu(end)];
        epCount = epCount + 1;
    end
    
    % Trim results
    endPts(epCount+1:end, :) = [];
    rayPts(rpCount+1:end, :) = [];
    obsInRange(obsInRangeCount+1:end, :) = [];
    zCells = unique(round(rayPts), 'rows');
end
