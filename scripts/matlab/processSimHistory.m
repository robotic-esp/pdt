function interpPlannerData = processSimHistory(filename, numExperiments, maxTime, interpTime)
    %interpPlannerData is :  numPlanners x 2 x numExperiments x interpolatedTimeSteps
    
%     %Read the raw data
%     rawData = csvread(filename, 0, 1);
% 
%     %Split into times and costs:
%     rawTime = rawData(1:2:size(rawData,1),:);
%     rawCost = rawData(2:2:size(rawData,1),:);
% 
%     %Be done with rawData
%     clear rawData;

    %Read the raw data
    [rawTime, rawCost] = rawHistoryFile(filename,2*maxTime);

    %Clean up erroneous values:
    rawTime(~isfinite(rawTime)) = nan;
    rawTime(rawTime == 0) = nan;

    rawCost(~isfinite(rawCost)) = inf;
    rawCost(rawCost == 0) = inf;

    %Calculate the number of planners:
    numTotalRows = size(rawTime,1);
    numTotalCols = size(rawTime,2);
    numPlanners = numTotalRows/numExperiments;
    
    if (mod(numTotalRows,numExperiments) ~= 0)
        error('Calculated a noninteger number of planners. The specified number of experiments or planners may been incorrect.');
    end

    %Allocate the plannerdata as planner x {time,cost} x iteration
    rawPlannerData = nan(numPlanners, 2, numExperiments, size(rawTime,2));

    %Now, make a 3D matrix of the raw data:
    for i = 1:numPlanners
        %Reshape into the planner data
        rawPlannerData(i,1,:,:) = reshape(rawTime(i:numPlanners:numTotalRows,:), [1, 1, numExperiments, numTotalCols]);
        rawPlannerData(i,2,:,:) = reshape(rawCost(i:numPlanners:numTotalRows,:), [1, 1, numExperiments, numTotalCols]);
    end
    
    %Now interpolate!
    interpTimes = (0:interpTime:maxTime)';
    
    interpPlannerData = nan(numPlanners, 2, numExperiments, size(interpTimes,1));
    
    for i = 1:numPlanners
        for j = 1:numExperiments
            %Get the time and data
            thisTime = squeeze(rawPlannerData(i,1,j,:));
            thisCost = squeeze(rawPlannerData(i,2,j,:));
            
            %Get the stuff that has no nans
            thisCleanTime = thisTime(isfinite(thisTime));
            thisCleanCost = thisCost(1:size(thisCleanTime,1), 1:size(thisCleanTime,2));
            
            if ~isscalar(thisCleanTime)
                interpPlannerData(i, 1, j, :) = interpTimes;
                try
                    interpData = interp1(thisCleanTime, thisCleanCost, interpTimes);
                catch me
                    interpData = interp1(thisCleanTime(1:end-1,1), thisCleanCost(1:end-1,1), interpTimes);
                end
                %In R2011a, interpolating INFs give NANs, but in R2014
                %they give INFs. Be consistent:
                interpData(~isfinite(interpData)) = inf;
                interpPlannerData(i, 2, j, :) = interpData;
            else
                interpPlannerData(i, 1, j, 1) = thisCleanTime;
                if isscalar(thisCleanCost)
                    interpPlannerData(i, 2, j, 1) = thisCleanCost;
                elseif isempty(thisCleanCost)
                    interpPlannerData(i, 2, j, 1) = inf;
                else
                    error('non scalar cost')
                end 
            end
        end
    end
end
