%New particle propagation
clc; clear all; close all
addpath("utils","images")
%%-----Initialize Run-----%%
seed = 0;
rng(seed) 
fileNS = sprintf('Example_%s',datestr(now,"mmddyy-HHMMSS"));
makeVideoLogic = false; saveDataLogic = false;

%%-----Define Initial Robot Parameters-----%%
robotVel     = 5;
angles       = -pi:pi/16:pi;
dt           = 0.1;
maxTurn      = pi/4;
maxRange     = 3;
epsilon      = 0.1;
initPoses    = [11 4 pi/2; 
                13 4 pi/2];
                %27 4 pi/2];
                % 29 4 pi/2];
numParticles = 2;
decay        = 0.2;
numFaults    = 1; 

%%-----Define Initial Environment-----%%
numTasks = 0;
obDist = 1; 
triggerRz = 50;

%%-----Start-----%%
InitializeSimEnvironment
triggerFault = randi([20,30]);
whichParticle = randi([2,numParticles]);
whichRobot = randi(numAgents);

%%-----Run-----%%
count = 0;
for i = 1:2000
    % fprintf('count: %d; iter: %d; avg compTime: %.2f\n',count,i,avg_timePerVehicle)
    tic;
    count = count + 1;
    connected_logic %Determine if robots are connected - if so, share info

    for k = 1:numAgents
        %Lidar Scan
        [zCells,rayPts,obsInRange{k},endPts{k}] = lidarSim(robots(k),map);
        %Updated up to here::::::
        first_covered
        rayPts_t{i,k} = rayPts;
        %Update known map
        robots(k).M0 = updateOccupancyMap(zCells,robots(k).M0,map);

        %Add attraction force at specified time
        if count>triggerRz
            robots(k).coeffs.otherBots = min((count-triggerRz)*1.01^(count-triggerRz),FC.obs-FC.obs/2);
            robots(k).coeffs.frontier = max(FC.frontier*0.99^(count-triggerRz),0);
        end
        
        if any(i==triggerFault)
            whichTrigger = find(i==triggerFault);
            robots(whichRobot(whichTrigger)).vel = robots(whichRobot(whichTrigger)).vel*(1-(whichParticle(whichTrigger)-1)*decay);
            robots(whichRobot(whichTrigger)).particleFollow = whichParticle(whichTrigger);
        end
    end
    for k = 1:numAgents
        fprintf('I am %d\n',k)
        %Compute particle movement
        particle_mimicry
    end
    for k = 1:numAgents
        %Calculate force if connected
        move_robot
        vel_t(i,k) = velAtt;
        ang_t(i,k) = angle;
        
    end
    % if i >=120
    %     s = 1;
    % end
    b = toc;
    %Combine total map
    storageLogic
    
    %Plot
    plotInline
    if connected && sum(sum(frontier))<1
        break
    end
end
F = F(1,1:i);
if makeVideoLogic 
    makeVideo(F,sprintf('%s',fileNS))
end
if saveDataLogic
    save(sprintf('%s.mat',fileNS))
end

