function rawPlannerData = processIterCostData(filename, numTrials, minCost)
    %rawPlannerData is :  numPlanners x {iter_number,cost} x numTrials x datapoints

    %Read the raw data
    [iterations, costs] = rawIterCostFile(filename, minCost);
    
    %Calculate the number of planners:
    numTotalRows = size(costs,1);
    numPlanners = numTotalRows/numTrials;
    numIterations = size(iterations,2);
    
    if (mod(numTotalRows,numTrials) ~= 0)
        error('Calculated a noninteger number of planner');
    end

    %Allocate the plannerdata as planner x {iter, cost} x numTrials
    rawPlannerData = nan(numPlanners, 2, numTrials, numIterations);

    %Now, make a 3D matrix of the raw data:
    for i = 1:numPlanners
        %Reshape into the planner data
        repmatIter = repmat(iterations, [numTrials, 1]);
        
        rawPlannerData(i,1,:,:) = reshape(repmatIter(i:numPlanners:numTotalRows,:), [1, 1, numTrials, numIterations]);
        rawPlannerData(i,2,:,:) = reshape(costs(i:numPlanners:numTotalRows,:), [1, 1, numTrials, numIterations]);
    end
end