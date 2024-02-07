%Connected logic
allPoses = [robots(:).pose];
distMat = squareform(pdist(allPoses(1:2,:)'))<2*maxRange;
bins = conncomp(graph(distMat));
connected = all(bins==1);
if connected
    count = 0;
    Mtot = zeros(size(robots(1).M0,1),numAgents);
    for k = 1:numAgents
        robots(k).particleFollow = 1;
        robots(k).particleGuess = ones(numAgents,1);
        Mtot(:,k) = robots(k).M0;
        robots(k).coeffs.otherBots = -1;
    end
    [~,I] = max(abs(Mtot-0.5),[],2);
    I = sub2ind(size(Mtot),(1:length(Mtot))',I);
    M1 = Mtot(I);
    [robots(:).M0] = deal(M1);
    for k = 1:numAgents
        for r = 1:numAgents
            for p = 1:numParticles
                for f = fn
                    robots(k).particles(r,p).(f{1}) = robots(r).(f{1});
                end
                robots(k).particles(r,p).vel = robots(r).vel*(1-(p-1)*decay);
            end
        end
    end
end
for k = 1:numAgents
    for r = 1:numAgents
        robots(k).guessLocs(:,r) = robots(k).particles(r,robots(k).particleGuess(r)).pose;
    end
end
