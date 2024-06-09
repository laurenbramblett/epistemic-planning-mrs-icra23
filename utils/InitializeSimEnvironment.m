%%-----Map-----%%
map.size        = [30,30]; 
map.M0          = zeros(map.size)+0.5;
map.M0([1:2 map.size(2)-1:map.size(2)],:) = 1;
map.M0(:,[1:2 map.size(1)-1:map.size(1)]) = 1;
[r_o, c_o]      = find(map.M0 == 1);
map.knownObs    = [c_o,r_o];
%Grid Points
[X,Y]           = meshgrid(1:map.size(1),1:map.size(2));
map.gridPoints  = [X(:),Y(:)];

%%-----Forces-----%%
FC           = struct;
FC.obs       = 10;
FC.frontier  = 3;
FC.otherBots = -1;
FC.goal      = 0;
FC.goalQueue = [];
FC.goalState = "";

%Goal forces
TC = struct;
TC.obs = 5;
TC.frontier = 0;
TC.otherBots = 0;
TC.goal = 1;
TC.goalQueue = [];
TC.goalState = "task";


%%-----Robots & Particles-----%%
initialPose = initPoses'; 
numAgents = size(initialPose,2);
for k = 1:numAgents
    robots(k).ID        = k;
    robots(k).pose      = initialPose(:,k);
    robots(k).depot     = initialPose(:,k);
    robots(k).numAgents = numAgents;
    robots(k).vel       = robotVel;
    robots(k).decay     = decay;
    robots(k).dt        = dt;
    robots(k).angles    = angles; 
    robots(k).maxRange  = maxRange; 
    robots(k).res       = maxRange*2;
    robots(k).guessLocs = initialPose;
    robots(k).M0        = map.M0(:);
    robots(k).maxTurn   = pi/4;
    robots(k).epsilon   = epsilon;
    robots(k).particleGuess = ones(numAgents,1);
    robots(k).coeffs    = FC;
    robots(k).goalQueue = [];
    robots(k).state     = "";
end

fn = fieldnames(robots(k))';

%Particle Inits
for k = 1:numAgents
    for r = 1:numAgents
        for p = 1:numParticles
            particles(k,r,p)      = robots(r);
            particles(k,r,p).vel  = robots(r).vel*(1-(p-1)*decay);
        end
    end
end

for k = 1:numAgents
    for r = 1:numAgents
        for p = 1:numParticles
            robots(k).particles(r,p) = particles(k,r,p);
        end
    end
end

%%-----Faults-----%%
for f = 1:numFaults
    triggerFault(f)     = randi([300 500]);
    triggerParticle(f)  = randi([2 numParticles]);
end
particleFollow = ones(1,numAgents);
velCats = [2,1.4,0.8];

%%-----Tasks-----%%
tasks         = ((map.size(1)-5)*rand(numTasks,2)+5);
completeRange = 1;

%%-----Obstacles-----%%
extraObs = 0;%randi([5,15]);
xObs     = round((45-5)*rand(extraObs,1)+5);
yObs     = round((45-5)*rand(extraObs,1)+5);
for e = 1:extraObs
    checkOb     = pdist2([xObs(e),yObs(e)],[xObs(e~=1:end),yObs(e~=1:end)])<maxRange*2*3/4;
    checkKnown  = pdist2([xObs(e),yObs(e)],obs)<maxRange*2*3/4;
    checkTarg   = pdist2([xObs(e),yObs(e)],tasks)<maxRange;
    sumCheck    = sum(checkOb) + sum(checkTarg) + sum(checkKnown);
    eCount = 0;
    while sumCheck>0
        xObs(e)     = round((45-5)*rand()+5);
        yObs(e)     = round((45-5)*rand()+5);
        checkOb     = pdist2([xObs(e),yObs(e)],[xObs(e~=1:end),yObs(e~=1:end)])<maxRange*2*2/4;
        checkKnown  = pdist2([xObs(e),yObs(e)],obs)<maxRange*2*2/4;
        checkTarg   = pdist2([xObs(e),yObs(e)],tasks)<maxRange;
        sumCheck    = sum(checkOb) + sum(checkTarg) + sum(checkKnown);   
        eCount      = eCount + 1;
    end
end
mapIdx = sub2ind(map.size,xObs,yObs);
map.M0(mapIdx) = 1;
[r_o, c_o] = find(reshape(map.M0,map.size) == 1);
map.obs = [c_o,r_o];

%%-----Plotting-----%%
[fImg, ~, alphachannel] = imread('fireImage.png');
cr                      = [0 0.4470 0.7410;
                           0.8500 0.3250 0.0980;
                           0.4940 0.1840 0.5560;
                           0.4660 0.6740 0.1880]; %Color pallete
firstCovered            = zeros(size(X(:)));
frontier                = logical(ones(length(X(:)),numAgents));

%%-----MPC-----%%
obNum = 10; %obstacles we care about 
N = 12;
mpcInit_v2


