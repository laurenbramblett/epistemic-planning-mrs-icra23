clc; clear all; close all
fp = "C:\Users\qbr5kx\OneDrive - University of Virginia\Desktop\UVA\PhD Scratch\DynamicSpringsExploration\SimulationWS";
%New particle propagation
for f = 0:2
    triggerFault1 = randi([100, 500]);
    triggerFault2 = randi([100, 500]);
    triggerParticle1 = randi([2 3]);
    triggerParticle2 = randi([2 3]);
for s = 0:12
%     if s < 6 && f == 0
%         continue
%     end
try
clearvars -except s f triggerFault1 triggerFault2 triggerParticle1 triggerParticle2 fp
velCats = [2,1.4,0.8];
seed = s;
InitializeSimEnvironment_MandatoryConnected
fprintf('Starting...Seed:%d; Tasks: %d; Faults: %d; ObsNum: %d; Time: %s\n',...
seed,numTasks,f,extraObs,datestr(now,'mm-dd-yy-HHMMSS'))
for i = 1:5000
%     fprintf('count: %d; iter: %d; avg compTime: %.2f\n',count,i,avg_timePerVehicle)
    tic;
    count = count + 1;
    for k = 1:agents
        connected = all(pdist2(pose(1:2,k)',pose(1:2,k~=1:end)')<2*maxRange);
        if pdist2(pose(1:2,k)',pose(1:2,k~=1:end)')<1
            agentFC{k}.otherBots = -1;
        elseif pdist2(pose(1:2,k)',pose(1:2,k~=1:end)')<maxRange
            agentFC{k}.otherBots = 0;
        else
            agentFC{k}.otherBots = 5;
        end
        if connected
            [~,I] = max(abs(M1-0.5),[],2);
            I = sub2ind(size(M1),(1:length(M1))',I);
            M1 = repmat(M1(I),1,agents);
            MGhost = M1;
            [r_o, c_o] = find(reshape(M1(:,k),mapSize) == 1);
            obs = [c_o,r_o];
            guessLocs(:,:,k) = pose;
            particles{k} = repmat(pose,1,1,numParticles,agents);
            Mpart(:,:,:,k) = repmat(M1(:,1),1,numParticles,agents);
            particleGuess = ones(agents,agents);
%             agentFC{k}.otherBots = -1;
%             [particleFC{k~=1:end,:,:}] = deal(agentFC{k});
            count = 0;
            vel(k,:) = realVel;
        else
%             agentFC{k}.otherBots = 0; 
%             if any(sum(idxF,1)<1)||any(isempty(idxF))
%                 agentFC{k}.otherBots = 2;
%             end

                
            if k == 1
                guessLocs(:,:,k) = [particles{k}(:,k,particleGuess(k,k~=1:end),k~=1:end),...
                                    particles{k}(:,k~=1:end,particleGuess(k,k~=1:end),k)];
            else
                guessLocs(:,:,k) = [particles{k}(:,k~=1:end,particleGuess(k,k~=1:end),k),...
                                    particles{k}(:,k,particleGuess(k,k~=1:end),k~=1:end)];
            end
        end
    end
    for k = 1:agents
        %Lidar Scan
        [zCells1,~,~,~] = lidarSim_connected(angles,guessLocs(:,k,k),obsReal,maxRange,res,mapSize);
        [zCells2,~,~,~] = lidarSim_connected(angles,guessLocs(:,k~=1:end,k),obs,maxRange,res,mapSize);
        zCells = [zCells1; zCells2];
        
        [zCellsTrue,rayPts,obsInRange{k},endPts{k}] = lidarSim_connected(angles,pose(:,k),obsReal,maxRange,res,mapSize);
        first_covered
        rayPts_t{i,k} = rayPts;
        %Update known map
        M1(:,k) = updateOccupancyMap_connected(zCellsTrue,M0(:,k),mapSize);
        MGhost(:,k) = updateOccupancyMap_connected(zCells,MGhost(:,k),mapSize);
        %Add attraction force at specified time
%         if count>200
%             agentFC{k}.otherBots = min((count-100)*1.001,3);
%         end
        if i==triggerFault1 && f>0
            realVel = [velCats(triggerParticle1),realVel(2)];
            particleGuess(1,1) = triggerParticle1;
        end
        if i == triggerFault2 && f>1
            realVel = [realVel(1),velCats(triggerParticle2)];
            particleGuess(2,2) = triggerParticle2;
        end
        minVel = min(realVel);
        realVel = repelem(min(realVel),1,agents);
        
    end
    for k = 1:agents
%         fprintf('I am %d\n',k)
        occMap = reshape(MGhost(:,k),mapSize);
 
        %Compute particle movement
        particle_mimicry_connected
    end
    for k = 1:agents
        %Calculate force if connected
        move_real_robot_connected
        vel_t(i,k) = velAtt;
        ang_t(i,k) = angle;
        
    end
    b = toc;
    %Combine total map
    storageLogic
    
%     %Plot
%     h1 = subplot(1,2,1);
%     plotInline
    if connected && sum(sum(frontier))<1
        break
    end
    
end

%     makeVideo(F,'SimulationWS/FrontAssignment_1Fault_ExperimentTest')

    yalmip('clear'); clearvars constraints controller u x0 x slack ops
    save(sprintf("%s\\SimResultsMandatoryConnected_seed%d_faults%d.mat",fp,seed,f))
    fprintf('Ending...Iters: %d at time %s \n',i,datestr(now,'mm-dd-yy-HHMMSS'))
catch
    yalmip('clear'); clearvars constraints controller u x0 x slack ops
    save(sprintf('%s\\SimResultsMandatoryConnected_seed%d_faults%dNOTCOMPLETE.mat',fp,seed,f))
    fprintf('Ending...Iters: %d at time %s NOT COMPLETE\n',i,datestr(now,'mm-dd-yy-HHMMSS'))
end
end
end