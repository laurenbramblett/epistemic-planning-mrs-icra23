distMat = squareform(pdist(robots(k).guessLocs(1:2,:)'))<3/2*maxRange;
bins = conncomp(graph(distMat));
particleSee = all(bins==1);
if particleSee && ~connected
    robots(k).particleGuess = min(robots(k).particleGuess + 1,numParticles); 
%     particleGuess(k,k) = min(particleGuess(k,k) + 1,numParticles); 
%     att = att*5;
end