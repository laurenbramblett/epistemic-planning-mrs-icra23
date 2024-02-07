%Plot Inline
%     h1 = subplot(1,2,1);
if i == 1
    plt.map = gobjects(1,1);
    plt.frontier = gobjects(1,numAgents);
    plt.rays = gobjects(1,numAgents);
    plt.poses = gobjects(1,numAgents);
    plt.beliefs = gobjects(numAgents,numParticles);
    plt.guesses = gobjects(1,1);
    plt.connection = gobjects(1,1);
    plt.anchor = gobjects(1);
    F = repmat(struct('cdata',[],'colormap',[]),1,2000);
end

if i == 1
    hold on;
    plt.map = imagesc(squeeze(M_t(:,:,1))); colormap(flipud(bone));
    axis xy; axis equal;
    for k = 1:numAgents
        frontierCoords = map.gridPoints(frontier(:,k),:);
        attractFront = frontierCoords(attractiveFront{k}(:,k),:);
        plt.frontier(k) = scatter(attractFront(:,1),attractFront(:,2),...
            'Marker','s','MarkerFaceColor',cr(k,:),'MarkerEdgeColor',cr(k,:),'MarkerFaceAlpha',0.1);
        plt.rays(k) = patch(endPts{k}(:,1),endPts{k}(:,2),cr(k,:),'FaceAlpha',0.1,'LineStyle','none');
        plt.poses(k) = scatter(pose_t(1,k,1),pose_t(2,k,1),36,'Marker','o','MarkerEdgeColor','k','MarkerFaceColor',cr(k,:));
        for p = 1:numParticles
            plt.beliefs(k,p) = scatter(robots(k).particles(k,p).pose(1),...
                                        robots(k).particles(k,p).pose(2),36,cr(k,:),...
                                        'MarkerEdgeColor','none','MarkerFaceColor',cr(k,:),...
                                        'MarkerFaceAlpha',0.5);
        end
    end
    plt.connection = plot(NaN, NaN, 'g--'); % Initialize with dummy data
    plt.guesses = plot(NaN, NaN, 'r--'); % Initialize with dummy data
    xlim([1 map.size(1)]);
    ylim([1 map.size(2)]);
    set(gcf,'color','w');
else
    % Update the map
    set(plt.map, 'CData', squeeze(M_t(:,:,i)));
    
    for k = 1:numAgents
        % Update frontiers and rays
        frontierCoords = map.gridPoints(frontier(:,k),:);
        attractFront = frontierCoords(attractiveFront{k}(:,k),:);
        set(plt.frontier(k), 'XData', attractFront(:,1), 'YData', attractFront(:,2));
        set(plt.rays(k), 'XData', endPts{k}(:,1), 'YData', endPts{k}(:,2));
        
        % Update poses
        set(plt.poses(k), 'XData', pose_t(1,k,i), 'YData', pose_t(2,k,i));
        
        % Update beliefs
        for p = 1:numParticles
            set(plt.beliefs(k,p), 'XData', robots(k).particles(k,p).pose(1),...
                                  'YData', robots(k).particles(k,p).pose(2));
        end
    end
    
    % Update connection or guesses based on 'connected' flag
    if connected
        set(plt.connection, 'XData', pose_t(1,:,i), 'YData', pose_t(2,:,i));
        set(plt.guesses, 'XData', [], 'YData', []); % Clear guesses if not needed
    else
        set(plt.guesses, 'XData', robots(1).guessLocs(1,:), 'YData', robots(1).guessLocs(2,:));
        set(plt.connection, 'XData', [], 'YData', []); % Clear connection if not connected
    end
end

title(sprintf('Time: $%.2f$s',i*dt),'interpreter','latex');

% Update legend only once outside the loop if it doesn't change
if i == 1
    % Initialize legend
    grayColor = [0.5 0.5 0.5];
    beliefsPlot = scatter(NaN,NaN,36,grayColor,'MarkerEdgeColor',grayColor,'MarkerFaceColor',grayColor,'MarkerFaceAlpha',0.5);
    frontierPlot = scatter(NaN,NaN,36,grayColor,'MarkerEdgeColor',grayColor,'MarkerFaceColor',grayColor,'Marker','s','MarkerFaceAlpha',0.1);
    anchorPlot = plot(NaN,NaN,'Color',grayColor,'LineStyle','--');
    for k = 1:numAgents
        names{k} = strcat("Agent ",num2str(k));
    end
    names{numAgents+1} = "Beliefs"; names{numAgents+2} = "Frontiers"; names{numAgents+3} = "Anchor";
    leg = legend([plt.poses,beliefsPlot,frontierPlot,anchorPlot],names,'Location','bestoutside');
    leg.ItemTokenSize = [6,18];
end

% Capture the frame
F(i) = getframe(gcf);

% Your existing time averaging logic remains unchanged
if i == 1
    avg_timePerVehicle = b/2;
else
    avg_timePerVehicle = (avg_timePerVehicle + b/2)/2;
end
