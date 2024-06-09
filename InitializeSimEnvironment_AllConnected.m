%Start
rng(seed)
%Lidar
angles = -pi:pi/16:pi; maxRange = 5; res = maxRange*2;
count = 0;
%Map initialize
mapSize = [50,50]; 
map = zeros(mapSize);
map([1:2 mapSize(2)-1:mapSize(2)],:) = 1;
map(:,[1:2 mapSize(1)-1:mapSize(1)]) = 1;
%
[r_o, c_o] = find(map == 1);
obs = [c_o,r_o];


[X,Y] = meshgrid(1:mapSize(1),1:mapSize(2));
gridPoints = [X(:),Y(:)];
count = 0;

%Robots
initialPose = [23 4 pi/2; 
               27 4 pi/2]'; 
bestGuess = [initialPose(:,2) initialPose(:,1)];
agents = size(initialPose,2);
guessLocs = repmat(initialPose,1,1,agents);
vel = [2, 2; 2, 2]; realVel = [2,2]; dt = 0.1;
pose = initialPose;
M0 = ones(length(gridPoints),agents)*0.5;
M0(map == 1) = 1;
M1 = M0; MGhost = M0;
avg_timePerVehicle = 0;
maxTurn = pi/4; epsilon = 0.3;

%Unknown tasks
numTasks = randi([3 7]);
tasks = [];
tasks = ((45-5)*rand(numTasks,2)+5);
completeRange = 1;

%Unknown obstacles
extraObs = randi([5,15]);
xObs = round((45-5)*rand(extraObs,1)+5);
yObs = round((45-5)*rand(extraObs,1)+5);
%For s<6&&f==0 maxRange*2*4/5 now maxRange*2
for e = 1:extraObs
    checkOb = pdist2([xObs(e),yObs(e)],[xObs(e~=1:end),yObs(e~=1:end)])<maxRange*2*3/4;
    checkKnown = pdist2([xObs(e),yObs(e)],obs)<maxRange*2*3/4;
    checkTarg = pdist2([xObs(e),yObs(e)],tasks)<maxRange;
    sumCheck = sum(checkOb) + sum(checkTarg) + sum(checkKnown);
    eCount = 0;
    while sumCheck>0
        xObs(e) = round((45-5)*rand()+5);
        yObs(e) = round((45-5)*rand()+5);
        checkOb = pdist2([xObs(e),yObs(e)],[xObs(e~=1:end),yObs(e~=1:end)])<maxRange*2*2/4;
        checkKnown = pdist2([xObs(e),yObs(e)],obs)<maxRange*2*2/4;
        checkTarg = pdist2([xObs(e),yObs(e)],tasks)<maxRange;
        sumCheck = sum(checkOb) + sum(checkTarg) + sum(checkKnown);   
        eCount = eCount + 1;
%         if eCount > 10000
%             fprintf('ImStuck')
%         end
    end
    
end
mapIdx = sub2ind(mapSize,xObs,yObs);
map(mapIdx) = 1;
%
[r_o, c_o] = find(map == 1);
obsReal = [c_o,r_o];

%Initialize particles
numParticles = 1;
Mpart = repmat(M1(:,1),1,numParticles,agents,agents);
% particles = zeros(3,numParticles,agents);
particleGuess = [1,1
                 1,1];
cr = [0 0.4470 0.7410;
      0.8500 0.3250 0.0980];

  
%Initialize force coefficients (FC)
initCoefficients 

%Initialize MPC
obNum = 10; %obstacles we care about 
N = 5;
mpcInit_v2
%Initialize Coverage
firstCovered = zeros(size(X(:)));
frontier = logical(ones(length(X(:)),agents));