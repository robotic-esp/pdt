function [succHandl, timeHandl] = plotTargetTimes(variateType, rawPlannerData, plannerNames, plannerColours, ignorePlanners, useMedian, yLimMargin)
    %rawPlannerData is :  numPlanners x {target,time} x numExperiments x numTargets
    
    %Bah
    vErrTick = 100;
    hErrTick = 0.005;
    labelFontSize = 14;
    legendFontSize = 12;
    numExp = size(rawPlannerData,3);
    numTargets = size(rawPlannerData,4);
    minNumNam = numExp/2;
    
    if size(rawPlannerData,1) ~= size(plannerNames,1)
        error('ASRL:plotTargetTimes:BadData', 'Number of names does not match number of planners in data');
    end

    %Preallocate:
    succHandles = nan(size(rawPlannerData,1),1);
    timeHandles = nan(size(rawPlannerData,1),1);
    plotPlanners = true(size(rawPlannerData,1),1);
    
    %Ranges:
    minPlotTime = inf;
    maxPlotTime = 0;
    minTargetVal = inf;
    maxTargetVal = 0;
    minTimeCI = zeros(size(rawPlannerData,1),1);
    maxTimeCI = zeros(size(rawPlannerData,1),1);
    

     %Configure:
	if useMedian == true
        plotAverageSTDErrorBars = false;
        if numExp == 50
            %See confidenceIntervalTest(n, i, j)
            %For 96.7%. (i.e., 95%)
%             lowIdx = 18;
%             highIdx = 32;

            %For 99.9%.
            lowIdx = 14;
            highIdx = 36;
            plotMedianCIErrorBars = true;
        elseif numExp == 100
            %See confidenceIntervalTest(n, i, j)
            %For 95%. 
    % 	      lowIdx = 40;
    %         highIdx = 60;
            %For 99.9%.
            lowIdx = 34;
            highIdx = 66;
            plotMedianCIErrorBars = true;
        else
            %Unknown number of experiments.
            warning('ASRL:plotTargetTimes', 'Error bars unsupported for the number of experiments.')
            plotMedianCIErrorBars = false;
        end
    else
        plotAverageSTDErrorBars = true;
        plotMedianCIErrorBars = false;
	end
    
    succHandl = figure;
    hold on;
    timeHandl = figure;
    hold on;
    for i = 1:size(rawPlannerData,1)

        skipPlanner = false;
        for j = 1:size(ignorePlanners,1)
            skipPlanner = skipPlanner || strcmp(plannerNames(i), ignorePlanners(j));
        end
        
        plotPlanners(i) = ~skipPlanner;

        %Get the target and time: 
        thisTarget = squeeze(rawPlannerData(i,1,1,:))'; %(1 x numTargets)
        thisTime = squeeze(rawPlannerData(i,2,:,:)); %(numExp x numTargets)
        
        
        if plotPlanners(i) == true && sum(sum(isfinite(thisTime))) > 0 && sum(sum(isfinite(thisTarget))) > 0
            %Calculate the min and max of the target values
            minTargetVal = min([minTargetVal, min(thisTarget)]);
            maxTargetVal = max([maxTargetVal, max(thisTarget)]);
            
            
            %Plot the success rate:
                successRate = 100*sum(isfinite(thisTime), 1)/numExp;
                        
                figure(succHandl);
        
                succHandles(i,1) = plot(thisTarget, successRate);
                                            
                set(succHandles(i,1), 'Color', plannerColours{i});
                set(succHandles(i,1), 'Marker', 'o');
                set(succHandles(i,1), 'LineStyle', '-');
                set(succHandles(i,1), 'LineWidth', 3);
            %End success plotting

            %Plot the timing results:
            
                %Calculate the median time targets that all experiment found a solution:
                if useMedian == true
                    %As non solutions are inf's, median will still work:
                    centralFullTime = median(thisTime, 1);
                else
                    %As non solutions are inf's, mean will not work, convert to nan's temporarily
                    thisNanTime = thisTime;
                    thisNanTime(~isfinite(thisNanTime)) = nan;
                    centralFullTime = mean(thisNanTime, 1);
                end

                %Update the limits from the full median:
                minPlotTime = min([minPlotTime, min(centralFullTime(isfinite(centralFullTime)))]);
                maxPlotTime = max([maxPlotTime, max(centralFullTime(isfinite(centralFullTime)))]);

                %Calculate the partial mean line, we don't have to do anything for mean
                if useMedian == false
                    partialTarget = thisTarget;
                    centralPartialTime = minNumNanMean(thisNanTime, 1, minNumNam, numExp);
                    
                    %Update the limits from partial median:
                    minPlotTime = min([minPlotTime, min(centralPartialTime(isfinite(centralPartialTime)))]);
                    maxPlotTime = max([maxPlotTime, max(centralPartialTime(isfinite(centralPartialTime)))]);
                else
                    partialTarget = [];
                    centralPartialTime = [];
                end

                if plotMedianCIErrorBars == true
                    %Sort the times for error bars:
                    sortTime = sort(thisTime, 1);

                    lowerBoundTime = centralFullTime - sortTime(lowIdx,:);
                    higherBoundTime = sortTime(highIdx,:) - centralFullTime;
                elseif plotAverageSTDErrorBars == true
                    lowerBoundTime = std(thisTime, 0, 1);
                    higherBoundTime = std(thisTime, 0, 1);
                end

                if (plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true)
                    %Update the limits with the full median CIs
                    minTimeCI(i,1) = min(centralFullTime - lowerBoundTime);
                    maxTimeCI(i,1) = max(centralFullTime + higherBoundTime);
                end

                %Now actually plot the timing results:
                figure(timeHandl);

                %Plot the incomplete trend
                tempHandl = plot(partialTarget, centralPartialTime);
                set(tempHandl, 'Color', plannerColours{i});
                set(tempHandl, 'LineStyle', '--');
                set(tempHandl, 'LineWidth', 3);

                %Plot the full trend
                timeHandles(i,1) = plot(thisTarget, centralFullTime);
                set(timeHandles(i,1), 'Color', plannerColours{i});
                set(timeHandles(i,1), 'LineStyle', '-');
                set(timeHandles(i,1), 'LineWidth', 3);

                %Plot the y error bars on the full median
                if plotMedianCIErrorBars == true || plotAverageSTDErrorBars == true

                    barCenters = centralFullTime;
                    barLowers = lowerBoundTime;
                    barUppers = higherBoundTime;

                    %Plot infinites finitely:
                    barUppers(~isfinite(barUppers)) = (1 + 1.1*yLimMargin)*maxPlotTime;

                    errHandle = errorbar(thisTarget, barCenters, barLowers, barUppers);
                    set(errHandle, 'Color', plannerColours{i});
                    set(errHandle, 'LineStyle', 'none');
%                     errorbar_tick(errHandle, vErrTick); %Set errorbar width
                end
        else
            %This planner has no data, do not plot:
            plotPlanners(i, 1) = false;
            
            %Announce
            fprintf('%s did not find a single solution. Not plotting.\n', plannerNames{i,1});
        end
    end
    
    
    %Remove all but one FMT* handle for succ/time.
    fmtFound = false;
    i = 1;
    while i <= size(succHandles,1)
        if strcmp('FMT', plannerNames{i}(1:3))                
            if fmtFound == true
                succHandles = [succHandles(1:i-1,1); succHandles(i+1:end,1)];
                timeHandles = [timeHandles(1:i-1,1); timeHandles(i+1:end,1)];
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
    handles = [succHandl;timeHandl];
    for i = 1:size(handles,1)
        figure(handles(i));
        
        grid on;
        box off;
        
        %Set the x limit (y is in if)
        xlim([0.9999*minTargetVal 1.0001*maxTargetVal]);
        
        if (strcmp(variateType, 'Map'))
            %Do nothing
        elseif strcmp(variateType, 'Gap')
             set(gca, 'XDir', 'reverse')
        elseif strcmp(variateType, 'Tol')
             set(gca, 'XDir', 'reverse')
             xlim([1 maxTargetVal + 0.01*(maxTargetVal-minTargetVal)]);
        else
            error('ASRL:plotTargetTimes:Unrecognized changed variable')
        end
        
        %Plot specific stuff. x/y label, y lim, and pbaspect
        if  handles(i) == succHandl    
            title('Runs solved vs. target', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            
            %Set the y axes
            ylabel('Success [\%]', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            ylim([0 101]);
        
            %Set the x axis
            xlabel('Target', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            
            %Set the ratio
            pbaspect([6 1 1]);
        elseif handles(i) == timeHandl
            title('Computational time vs. target', 'FontSize', labelFontSize, 'Interpreter', 'latex');        
        
            %Set the y axis
            if useMedian == true
                ylabel('Median computational time [s]', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            else
                ylabel('Mean computational time [s]', 'FontSize', labelFontSize, 'Interpreter', 'latex');
            end
            
            %Decide on y limits, we pick the first CI bound that is within
            %the provided limit of the medians
            yMin = min([minTimeCI(minTimeCI >= (1 - yLimMargin)*minPlotTime); minPlotTime]); %No minPlotTime buffer
            yMax = max([maxTimeCI(maxTimeCI <= (1 + yLimMargin)*maxPlotTime); (1 + 0.5*yLimMargin)*maxPlotTime]);
            
            if isinf(yMin)
                yMin = 0;
            end
            
            ylim([yMin - 0.01*mean([minPlotTime, maxPlotTime]) yMax + 0.01*mean([minPlotTime, maxPlotTime])]);
        
            %Set the scale
            set(gca,'YScale','log');
            set(gca,'YMinorGrid','on')

            %Set the ratio
            pbaspect([3 1 1]);
        else
            error('Unacceptable handle')
        end
        
        %Set the legends:
        if handles(i) == succHandl
            lHandle = legend(succHandles(plotPlanners), plannerNames(plotPlanners));
%            lHandle = gridLegend(succHandles(plotPlanners), 3, plannerNames(plotPlanners), 'FontSize', legendFontSize, 'Interpreter', 'latex', 'Orientation', 'Horizontal', 'Location', 'SouthOutside');
        elseif handles(i) == timeHandl
            lHandle = legend(timeHandles(plotPlanners), plannerNames(plotPlanners));
%            lHandle = gridLegend(timeHandles(plotPlanners), 3, plannerNames(plotPlanners), 'FontSize', legendFontSize, 'Interpreter', 'latex', 'Orientation', 'Horizontal', 'Location', 'SouthOutside');
        else
            error('Unacceptable handle')
        end
        
        %Set the legend font, basic position, etc
        set(lHandle, 'FontSize', legendFontSize);
        set(lHandle, 'Interpreter', 'latex');
        set(lHandle, 'Orientation', 'Vertical');
        set(lHandle, 'Location', 'SouthOutside');
        
% %         %Plot specific stuff. label fudge
% %         if  handles(i) == succHandl
% %             %Fudge vertical position
% %             lPos = get(lHandle, 'OuterPosition');
% %             set(lHandle, 'OuterPosition', [lPos(1) lPos(2)-0.23 lPos(3:4)] )
% %         elseif handles(i) == timeHandl
% %             %Fudge vertical position
%             lPos = get(lHandle, 'OuterPosition');
%             set(lHandle, 'OuterPosition', [lPos(1) lPos(2)-0.1 lPos(3:4)] )
% %         else
% %             error('Unacceptable handle')
% %         end

    end
end