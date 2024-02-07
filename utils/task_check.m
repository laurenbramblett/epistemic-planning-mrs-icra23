%Goal Construction
taskSpot = 0;
if ~isempty(tasks)
    taskDist = pdist2(robots(k).pose(1:2)',tasks);
    %Check if tasks are accomplished
    taskCheckLogic = taskDist<completeRange;
    tasks(taskCheckLogic,:) = [];
    taskDist = pdist2(robots(k).pose(1:2)',tasks);
    if sum(taskCheckLogic)>0
        robots(k).coeffs.goalState = "";
        taskToAgent = pdist2(robots(k).pose(1:2)',robots(k).coeffs.goalQueue)<completeRange;
        robots(k).coeffs.goalQueue(taskToAgent,:) = [];
    end
    %Update task distance
    taskDist = pdist2(robots(k).pose(1:2)',tasks);
    if sum(taskCheckLogic)>0 && ~isempty(tasks)
        taskSpot = taskDist<maxRange;
    elseif ~isempty(tasks)
        taskSpot = taskDist<maxRange;
    end
end

if sum(taskSpot)>0 
    robots(k).coeffs.goalQueue = unique([robots(k).coeffs.goalQueue; tasks(taskSpot,:)],'rows');
end
if size(robots(k).coeffs.goalQueue,1)>0
    taskDists = pdist2(robots(k).pose(1:2)',robots(k).coeffs.goalQueue);
    [distChoice,idxChoice] = min(taskDists);
    taskChoice = robots(k).coeffs.goalQueue(idxChoice,:);
    goal = taskChoice;
end

if connected
    agArr = [robots(:).coeffs];
    allPoses = [robots(:).pose];
    allGoals = [agArr(:).goalQueue]; gLen = length(allGoals);
    if gLen>2
        allGoals = reshape(allGoals,gLen/2,2)';
        allGoals = unique(allGoals,'rows');
    end
    [robots(:).coeffs] = deal(FC);
    if ~isempty(allGoals)
        for t = 1:size(allGoals,1)
            cTask = allGoals(t,:);
            [~,taskIdx] = min(pdist2(cTask,allPoses(1:2,:)'));
            robots(taskIdx).coeffs = TC;
            robots(taskIdx).coeffs.goalQueue = [robots(k).coeffs.goalQueue; cTask];
            for r = 1:numAgents
                for p = 1:numParticles
                    robots(r).particles(taskIdx,p).coeffs = robots(taskIdx).coeffs;
                end
            end
        end
    end
    for a = 1:numAgents
        if isempty(robots(a).coeffs.goalQueue)
            robots(a).coeffs = FC;
        end
    end
end

