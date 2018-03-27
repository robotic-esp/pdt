function [succHandl, histHandl, solnRates] = plotSimHistory(interpPlannerData, plannerNames, plannerColours, plannerLines, plotTime, ignorePlanners, useMedian, unsolvedNan, plotFailures, plotInfiniteCI, yLimMargin)
    %interpPlannerData is :  numPlanners x 2 (time, cost) x numExperiments x interpolatedTimeSteps
    %unsolvedNan = false treates unsolved as having infinite cost and
    %calculates the median appropriately.
    
    if (nargin ~= 11)
        error ('ASRL:plotSimHistory', 'The function signature of plotSimHistory has changed.')
    end
    
    %Bah
    vErrTick = 100;
    hErrTick = 0.005;
    labelFontSize = 14;
    legendFontSize = 12;
    numExp = size(interpPlannerData,3);
    minNumNam = numExp/2;
    
    %This is a bit of a hacky way to get the interpolated time:
    interpTime = squeeze(nanmedian(nanmedian(squeeze(interpPlannerData(:,1,:,:)))));
    interpTime(1) = 0;
    
    if size(interpPlannerData,1) ~= size(plannerNames,1)
        error('ASRL:plotSimHistory:BadData', 'Number of names does not match number of planners in data');
    end

    %Preallocate:
    solnRates = nan(size(interpPlannerData,1), size(interpPlannerData,4));
    succHandles = nan(size(interpPlannerData,1),1);
    histHandles = nan(size(interpPlannerData,1),1);
    histErrBarHandles = [];
    plotPlanners = true(size(interpPlannerData,1),1);
    
    %Ranges:
    minPlotTime = inf;
    maxPlotTime = min(plotTime, max(max(max(interpPlannerData(:,1,:,:)))));
    minHistCost = inf;
    maxHistCost = 0;
    minHistCostCI = zeros(size(interpPlannerData,1),1);
    maxHistCostCI = zeros(size(interpPlannerData,1),1);
    

     %Configure:
	if useMedian == true
        plotAverageSTDErrorBars = false;
        if numExp == 50
            %See confidenceIntervalTest(n, i, j)
%             %For 96.7%. (i.e., 95%)
%             lowIdx = 18;
%             highIdx = 33;
            %For 99.9%. (i.e., 99%)
            lowIdx = 14;
            highIdx = 37;
            plotMedianCIErrorBars = true;
        elseif numExp == 100
            %See confidenceIntervalTest(n, i, j)
%             %For 96.5 (i.e., 95%).
%             lowIdx = 40;
%             highIdx = 61;
            %For 99.9% (i.e., 99%)
            lowIdx = 34;
            highIdx = 67;
            plotMedianCIErrorBars = true;
        elseif numExp == 200
            %See confidenceIntervalTest(n, i, j)
%             %For 96.0% (i.e., 95%). 
%             lowIdx = 86;
%             highIdx = 115;
            %For 99.1% (i.e., 99%).
            lowIdx = 82;
            highIdx = 119;
            plotMedianCIErrorBars = true;
        elseif numExp == 250
            %See confidenceIntervalTest(n, i, j)
%             %For 95.0% (i.e., 95%). 
%             lowIdx = 110;
%             highIdx = 141;
            %For 99.1% (i.e., 99%).
            lowIdx = 105;
            highIdx = 146;
%             %For 99.92% (i.e., 99.9%)
%             lowIdx = 99;
%             highIdx = 152;
            plotMedianCIErrorBars = true;
        elseif numExp == 300
            %See confidenceIntervalTest(n, i, j)
%             %For 95.7% (i.e., 95%). 
%             lowIdx = 133;
%             highIdx = 168;
            %For 99.1% (i.e., 99%).
            lowIdx = 128;
            highIdx = 173;
            plotMedianCIErrorBars = true;
        elseif numExp == 500
            %See confidenceIntervalTest(n, i, j)
%             %For 95.6% (i.e., 95%). 
%             lowIdx = 228;
%             highIdx = 273;
            %For 99.1% (i.e., 99%).
            lowIdx = 221;
            highIdx = 280;
            plotMedianCIErrorBars = true;
        else
            %Unknown number of experiments.
            warning('Error bars unsupported for the number of experiments.')
            plotMedianCIErrorBars = false;
        end
    else
        plotAverageSTDErrorBars = true;
        plotMedianCIErrorBars = false;
	end
    
    succHandl = figure;
    hold on;
    histHandl = figure;
    hold on;
    for i = 1:size(interpPlannerData,1)

        skipPlanner = false;
        for j = 1:size(ignorePlanners,1)
            skipPlanner = skipPlanner || strcmp(plannerNames(i), ignorePlanners(j));
        end
        
        plotPlanners(i) = ~skipPlanner;

        %Get the time and cost:
        thisTime = squeeze(interpPlannerData(i,1,:,:));
        thisCost = squeeze(interpPlannerData(i,2,:,:));
        
        %If there is only 1 experiment, squeeze will create an tx1 vector
        %(instead of 1xt) fix that.
        if size(interpPlannerData,3) == 1
            warning('Only 1 experiment. Reshaping the vectors. This may not work correctly.')
            thisTime = reshape(thisTime, 1, size(thisTime,1));
            thisCost = reshape(thisCost, 1, size(thisCost,1));
        end
        
        if unsolvedNan == true
            thisCost(isinf(thisCost)) = nan;
        end
        
        if plotPlanners(i) == true && (plotFailures || (sum(sum(isfinite(thisCost))) > 0 && sum(sum(isfinite(thisTime))) > 0))

            %Calculate if this is scalar:
            isScalar = sum(sum(isfinite(thisTime))) == size(thisTime,1);
            
            %Find the initial solution and cost:
            initSolnTime = inf(size(thisTime,1),1);
            initSolnCost = inf(size(thisCost,1),1);
            
            for j = 1:size(thisTime,1)
                idx = find(isfinite(thisCost(j,:)),1);
                
                if (~isempty(idx))
                    initSolnTime(j) = thisTime(j,idx);
                    initSolnCost(j) = thisCost(j,idx);
                end
                %No else, leave as INFs
            end

            %Calculate the central value:
            if useMedian == true
                centralInitSolnTime = median(initSolnTime);
                centralInitSolnCost = median(initSolnCost);
                fprintf('%s: median solution time and cost: %1.4fs, %1.4f\n', plannerNames{i}, centralInitSolnTime, centralInitSolnCost);
            else
                centralInitSolnTime = mean(initSolnTime);
                centralInitSolnCost = mean(initSolnCost);
                fprintf('%s: mean solution time and cost: %1.4fs, %1.4f\n', plannerNames{i}, centralInitSolnTime, centralInitSolnCost);
            end
                

            %Sort:
            sortInitSolnTime = sort(initSolnTime, 1);
            sortInitSolnCost = sort(initSolnCost, 1);

            %Update limits:
            if isfinite(min(sortInitSolnTime))
                minPlotTime = min( [minPlotTime, min(sortInitSolnTime)] );
            end
            if isfinite(centralInitSolnCost)
                minHistCost = min( [minHistCost, centralInitSolnCost] );
                maxHistCost = max( [maxHistCost, centralInitSolnCost] );
            end


            %Plot the success rate:
                %This is just the index of sort vector plotted against the sorted time.
                figure(succHandl);

                %First, get the finite solution times:
                succTime = sortInitSolnTime( isfinite(sortInitSolnTime) );
                
                %Preallocate the % success to 0:
                succPerc = zeros(size(interpTime));
                
                %Iterate through each succTime to update the succPerc
                for t = 1:size(succTime,1)
                    succPerc( interpTime >= succTime(t) ) = 100*t/numExp;
                end
                
                solnRates(i,:) = succPerc;
                                
                %Then plot the padded time as x and the count as y;
                succHandles(i,1) = plot(interpTime, succPerc);
                set(succHandles(i,1), 'Color', plannerColours{i});
                set(succHandles(i,1), 'LineStyle', plannerLines{i});
                set(succHandles(i,1), 'LineWidth', 3);
            %End success plotting

            %Plot the history:
                if plotMedianCIErrorBars == true
                    %Calculate the initial solution boundaries:
                    lowerBoundInitTime = centralInitSolnTime - sortInitSolnTime(lowIdx,:);
                    higherBoundInitTime = sortInitSolnTime(highIdx,:) - centralInitSolnTime;
                    lowerBoundInitCost = centralInitSolnCost - sortInitSolnCost(lowIdx,:);
                    higherBoundInitCost = sortInitSolnCost(highIdx,:) - centralInitSolnCost;
                    
                elseif plotAverageSTDErrorBars == true
                    %Calculate the initial solution boundaries:
                    lowerBoundInitTime = min( [centralInitSolnTime, std(initSolnTime, 0, 1)] ); %Limit the lowerBoundInitTime to the centralInitSolnTime so that the error bar is at 0.
                    higherBoundInitTime = std(initSolnTime, 0, 1);
                    lowerBoundInitCost = std(initSolnCost, 0, 1);
                    higherBoundInitCost = std(initSolnCost, 0, 1);                    
                end
                
                if (plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true)
                    %Update the limits:
                    %With STD we can go negative...
                    if (centralInitSolnTime - lowerBoundInitTime) > 0
                        minPlotTime = min([minPlotTime, centralInitSolnTime - lowerBoundInitTime]);
                    end
                    
                    
                    minHistCostCI(i,1) = centralInitSolnCost - lowerBoundInitCost;
                    maxHistCostCI(i,1) = centralInitSolnCost + higherBoundInitCost;
                end
                

                %If this is a time history, calculate the median for the part of the history with >=50% success:
                if ( ~isScalar )
                    %Calculate the median time for the time steps that all experiments
                    %have a solution
                    if useMedian == true
                        centralFullSolnTime = median(thisTime, 1);
                        centralFullSolnCost = median(thisCost, 1);
                    else
                        centralFullSolnTime = mean(thisTime, 1);
                        centralFullSolnCost = mean(thisCost, 1);
                    end                        
                    
                    %Update the limits from the full median:
                    if any(any(isfinite(centralFullSolnCost)))
                        minHistCost = min([minHistCost, min(centralFullSolnCost(isfinite(centralFullSolnCost)))]);
                        maxHistCost = max([maxHistCost, max(centralFullSolnCost(isfinite(centralFullSolnCost)))]);
                    end
                    
                    %Calculate the partial median line
                    if useMedian == true
                        centralPartialSolnTime = median(thisTime, 1);
                        centralPartialSolnCost = minNumNanMedian(thisCost, 1, minNumNam, size(thisCost,1));

                    else
                        centralPartialSolnTime = mean(thisTime, 1);
                        centralPartialSolnCost = minNumNanMean(thisCost, 1, minNumNam, size(thisCost,1));
                    end

                    
                    %Update the limits from partial median:
                    if any(any(isfinite(centralPartialSolnCost)))
                        minHistCost = min([minHistCost, min(centralPartialSolnCost(isfinite(centralPartialSolnCost)))]);
                        maxHistCost = max([maxHistCost, max(centralPartialSolnCost(isfinite(centralPartialSolnCost)))]);
                    end
                    
                    if plotMedianCIErrorBars == true
                        %Sort the times and cost for error bars:
                        sortHistCost = sort(thisCost, 1);
                
                        lowerBoundHistCost = centralFullSolnCost - sortHistCost(lowIdx,:);
                        higherBoundHistCost = sortHistCost(highIdx,:) - centralFullSolnCost;
                        
                    elseif plotAverageSTDErrorBars == true
                        lowerBoundHistCost = std(thisCost, 0, 1);
                        higherBoundHistCost = std(thisCost, 0, 1);

                    end
                    
                    if (plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true)
                        %Update the limits with the full median CIs
                        minHistCostCI(i,1) = min(centralFullSolnCost - lowerBoundHistCost);
                        maxHistCostCI(i,1) = max(centralFullSolnCost + higherBoundHistCost);

                    end
                end

                %Now plot history:
                figure(histHandl);
                
                %The initial solution first:
                histHandles(i,1) = plot(centralInitSolnTime, centralInitSolnCost);
                set(histHandles(i,1), 'Color', plannerColours{i});
                set(histHandles(i,1), 'LineStyle', 'none');
                set(histHandles(i,1), 'Marker', '.');
                set(histHandles(i,1), 'MarkerSize', 15);
                
                %Plot both error bars on the median initial solution
                if plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true
                    %The min, 2*maxHistCost remaps infinite costs to a finite value off the plot...
                    errHandle = errorbar(centralInitSolnTime, centralInitSolnCost, lowerBoundInitCost, min( [higherBoundInitCost, (1 + 1.1*yLimMargin)*maxHistCost] ));
                    set(errHandle, 'Color', plannerColours{i});
                    set(errHandle, 'LineStyle', 'none');
                    
                    %Store the handle to resize later:
                    histErrBarHandles = [histErrBarHandles errHandle];
                    
                    errHandle = herrorbar(centralInitSolnTime, centralInitSolnCost, lowerBoundInitTime, min( [higherBoundInitTime, 1.1*maxPlotTime] ), 'k', hErrTick); %'k' is a place  holder
                    set(errHandle(1), 'Color', plannerColours{i});
                    set(errHandle(2), 'LineStyle', 'none');
                    %DO NOT try and set the herrorbar height with errobar_tick, it will actually change the width (i.e., length)
                end
                
                if ~isScalar
                    %Plot the incomplete trend
                    tempHandl = plot(centralPartialSolnTime, centralPartialSolnCost);
                    set(tempHandl, 'Color', plannerColours{i});
                    set(tempHandl, 'LineStyle', '--');
                    set(tempHandl, 'LineWidth', 3);

                    %Plot the full trend
                    histHandles(i,1) = plot(centralFullSolnTime, centralFullSolnCost);
                    set(histHandles(i,1), 'Color', plannerColours{i});
                    set(histHandles(i,1), 'LineStyle', plannerLines{i});
                    set(histHandles(i,1), 'LineWidth', 3);

                    %Plot the y error bars on the full median where they are
                    %finite
                    if plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true
                        minBarIndex = find(isfinite(centralFullSolnCost), 1);
                        minBarTime = centralFullSolnTime(:,minBarIndex);
                        barInterval = 1;%find(isfinite(centralFullSolnCost), 1);
                        for logIdx = floor(log10(minBarTime)):floor(log10(max(centralFullSolnTime)))
                            for idx = 1:10
                                foundIdx = find(abs(centralFullSolnTime - idx*10^logIdx) < 1e-9);
                                if foundIdx > max(barInterval)
                                    barInterval = [barInterval  foundIdx];
                                end
                            end
                        end
                        
                        barTimes = centralFullSolnTime(:,barInterval);
                        barCentres = centralFullSolnCost(:,barInterval);
                        barLowers = lowerBoundHistCost(:,barInterval);
                        barUppers = higherBoundHistCost(:,barInterval);
                                                
                        %%Plot infinites finitely:
                        if (plotInfiniteCI)
                            barUppers(~isfinite(barUppers)) = (1 + 1.1*yLimMargin)*maxHistCost;
                        end
                        
                        %%Mask off any infinities:
                        barMask = isfinite(barCentres) & isfinite(barLowers) & isfinite(barUppers);
                        
                        errHandle = errorbar(barTimes(barMask), barCentres(barMask), barLowers(barMask), barUppers(barMask));
                        set(errHandle, 'Color', plannerColours{i});
                        set(errHandle, 'LineStyle', 'none');
                        
                        %It can happen that, even though we have some
                        %solutions, we never have enough to have a median
                        %line. In which case, the error handles are empty
                        if (~isempty(errHandle))
                            %Store the handle to resize later:
                            histErrBarHandles = [histErrBarHandles errHandle];
                        end
                    end
                end
            %End history plotting
            
            %Announce the quality of the final path
            if ( ~isScalar )
                if useMedian == true
                    fprintf('%s: median solution cost after %1.4fs: %1.4f\n', plannerNames{i}, centralFullSolnTime(end), centralFullSolnCost(end));
                else
                    fprintf('%s: mean solution cost %1.4fs: %1.4f\n', plannerNames{i}, centralFullSolnTime(end), centralFullSolnCost(end));
                end
            else
                if useMedian == true
                    fprintf('%s: median solution cost after %1.4fs: %1.4f\n', plannerNames{i}, centralInitSolnTime, centralInitSolnCost);
                else
                    fprintf('%s: mean solution cost %1.4fs: %1.4f\n', plannerNames{i}, centralInitSolnTime, centralInitSolnCost);
                end
            end

            
        else
            %This planner has no data, do not plot:
            plotPlanners(i, 1) = false;
            
            %Announce
            fprintf('%s did not find a single solution. Not plotting.\n', plannerNames{i,1});
        end
    end
    
    
    %Remove all but one FMT* handle for succ/hist.
    allNames = plannerNames;
    fmtFound = false;
    i = 1;
    while i <= size(histHandles,1)
        if strcmp('FMT', plannerNames{i}(1:3))                
            if fmtFound == true
                succHandles = [succHandles(1:i-1,1); succHandles(i+1:end,1)];
                histHandles = [histHandles(1:i-1,1); histHandles(i+1:end,1)];
                plannerNames = [plannerNames(1:i-1,1); plannerNames(i+1:end,1)];
                plotPlanners = [plotPlanners(1:i-1,1); plotPlanners(i+1:end,1)];
                i = i - 1;
            else
                fmtFound = true;
                plannerNames{i} = 'FMT*';
            end
        end
        i = i + 1;
    end
    
    %Configure the plots
    handles = [succHandl;histHandl];
    for i = 1:size(handles,1)
        figure(handles(i));
        
        %Set the scale
        set(gca,'XScale','log');
        
        grid on;
        
        %Set the x limit (y is in if)
        xlim([minPlotTime maxPlotTime]);
        
        %Label the x axes (x is in if)
        xlabel('Computational time [s]', 'FontSize', labelFontSize, 'Interpreter', 'latex');
        
        %Plot specific stuff. y label, y lim, and pbaspect
        if  handles(i) == succHandl    
            title('Runs solved vs. time', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            
            %Set the y axis
            ylabel('Success [\%]', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            ylim([0 101]);

            %Set the ratio
            pbaspect([4 1 1]); %ICRA/IROS: [6 1 1]
        elseif handles(i) == histHandl
            title('Solution cost vs. time', 'FontSize', labelFontSize, 'Interpreter', 'latex');        
        
            %Set the y axis
            if useMedian == true
                ylabel('Median solution cost', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            else
                ylabel('Mean solution cost', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            end
            
            %Decide on y limits, we pick the first CI bound that is within
            %the provided limit of the medians
            yMin = min([minHistCostCI(minHistCostCI >= (1 - yLimMargin)*minHistCost); minHistCost]); %No minHistCost buffer
            yMax = max([maxHistCostCI(maxHistCostCI <= (1 + yLimMargin)*maxHistCost); (1 + 0.5*yLimMargin)*maxHistCost]);
            
            if isinf(yMin)
                yMin = 0;
            end
            
            ylim([yMin - 0.01*mean([minHistCost, maxHistCost]) yMax + 0.01*mean([minHistCost, maxHistCost])]);
            
            %Set the ratio
            pbaspect([3 1 1]); %ICRA/IROS: [3 1 1]
            
            %Iterate through the error bar handles and fix them.
            for j = 1:length(histErrBarHandles)
%                 errorbar_tick(histErrBarHandles(j), vErrTick); %Set errorbar width
                removeErrorBarCaps(histErrBarHandles(j));
            end
        else
            error('Unacceptable handle')
        end
        
        %Set the legends:
        if handles(i) == succHandl
            lHandle = legend(succHandles(plotPlanners), plannerNames(plotPlanners));
%            lHandle = gridLegend(succHandles(plotPlanners), 5, plannerNames(plotPlanners), 'FontSize', legendFontSize, 'Interpreter', 'latex', 'Orientation', 'Horizontal', 'Location', 'SouthOutside');
        elseif handles(i) == histHandl
            lHandle = legend(histHandles(plotPlanners), plannerNames(plotPlanners));
%            lHandle = gridLegend(histHandles(plotPlanners), 5, plannerNames(plotPlanners), 'FontSize', legendFontSize, 'Interpreter', 'latex', 'Orientation', 'Horizontal', 'Location', 'SouthOutside');
        else
            
            error('Unacceptable handle')
        end
        
        %Set the legend font, basic position, etc
        set(lHandle, 'FontSize', legendFontSize);
        set(lHandle, 'Interpreter', 'latex');
        set(lHandle, 'Orientation', 'Horizontal');
        set(lHandle, 'Location', 'SouthOutside');
        
%         %Plot specific stuff. label fudge
%         if  handles(i) == succHandl
%             %Fudge vertical position
%             lPos = get(lHandle, 'OuterPosition');
%             set(lHandle, 'OuterPosition', [lPos(1) lPos(2)-0.23 lPos(3:4)] )
%         elseif handles(i) == histHandl
%             %Fudge vertical position
%             lPos = get(lHandle, 'Position');
%             set(lHandle, 'Position', [lPos(1) lPos(2)-0.1 lPos(3:4)] )
%         else
%             error('Unacceptable handle')
%         end

    end
end