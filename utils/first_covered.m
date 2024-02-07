if ~isempty(zCells)
    zCellIdx = sub2ind(map.size,zCells(:,2),zCells(:,1));
    for z = 1:size(zCellIdx,1)
        if firstCovered(zCellIdx(z)) <1
            firstCovered(zCellIdx(z)) = k;
        end
    end
end