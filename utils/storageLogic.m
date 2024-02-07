%Storage Logic
%Combine total map
for r = 1:numAgents
    M1(:,r) = robots(r).M0(:);
end
[~,I] = max(abs(M1-0.5),[],2);
I = sub2ind(size(M1),(1:length(M1))',I);
Mtot = reshape(M1(I),map.size);
M_t(:,:,i) = Mtot;
pose_t(:,:,i) = [robots(:).pose];
frontierAll_t(:,:,i) = max(frontier,[],2);
frontier_t{i} = frontier;
%     perc_expl{seed+1}(i) = sum(sum(abs(M1(idx)-0.5)>0.45 &abs(M1(idx)-0.5)<0.55))/total_expl_denom;
frontier_ghost_t(:,:,i) = frontier;
particles_t{i} = [robots(:).particles];
anchorLocs_t{i} = robots(1).guessLocs;
attractiveFront_t{i} = attractiveFront;
endPts_t{i} = endPts;
particleGuess_t{i} = robots(1).particleGuess;
tasks_t{i} = tasks;
firstCovered_t{i} = firstCovered;
M1_t{i} = M1;