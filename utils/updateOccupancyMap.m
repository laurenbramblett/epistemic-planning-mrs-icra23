function M1 = updateOccupancyMap(zCells,M0,map)
    % Calculate the probability of being occupied
    prob = zeros(size(zCells,1),1);
    prob(zCells(:,3)<1) = 0.14;
    prob(zCells(:,3)>=1) = 0.86;
    IDx = sub2ind(map.size,zCells(:,2),zCells(:,1));

    M0(IDx) = prob.*M0(IDx)./(prob.*M0(IDx)+(1-prob).*(1-M0(IDx)));
    M1 = M0;
end
