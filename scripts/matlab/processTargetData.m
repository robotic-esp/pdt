function rawPlannerData = processTargetData(filename, numExperiments, firstTarget)
    %rawPlannerData is :  numPlanners x {target,time} x numExperiments x numTargets

    %Read the raw data
    [targets, rawTime] = rawTargetFile(filename);
    
    subTime = rawTime(:,firstTarget:end);
    subTargets = targets(firstTarget:end);
    
    numTargets = size(subTargets,2);

    %Clean up erroneous values:
    subTime(~isfinite(subTime)) = inf;
    subTime(subTime == 0) = inf;

    %Calculate the number of planners:
    numTotalRows = size(subTime,1);
    numPlanners = numTotalRows/numExperiments;
    
    if (mod(numTotalRows,numExperiments) ~= 0)
        error('Calculated a noninteger number of planner');
    end

    %Allocate the plannerdata as planner x {target, time} x numExperiments x numTargets
    rawPlannerData = nan(numPlanners, 2, numExperiments, numTargets);

    %Now, make a 3D matrix of the raw data:
    for i = 1:numPlanners
        %Reshape into the planner data
        repmatTarget = repmat(subTargets, [numExperiments, 1]);
        
        rawPlannerData(i,1,:,:) = reshape(repmatTarget, [1, 1, numExperiments, numTargets]);
        rawPlannerData(i,2,:,:) = reshape(subTime(i:numPlanners:numTotalRows,:), [1, 1, numExperiments, numTargets]);
    end
end
