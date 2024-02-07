%Initialize Structures for coefficients
FC = struct;
FC.obs = -100;
FC.frontier = 2.5;
FC.otherBots = 0;
FC.goal = 0;
FC.goalQueue = [];
FC.goalState = "";

% TC = struct;
% TC.obs = -100;
% TC.frontier = 0;
% TC.otherBots = 0;
% TC.goal = 2.5;
% TC.goalQueue = [];
% TC.goalState = "task";

agentFC = cell(agents,1);
[agentFC{:}] = deal(FC);

particleFC = cell(agents,agents,numParticles);
[particleFC{:}] = deal(FC);





