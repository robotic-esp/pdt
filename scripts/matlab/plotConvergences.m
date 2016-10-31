function [rateHandles, diffHandles] = plotConvergences(rawPlannerData, plannerNames, dimension, minimumCost, trialColour, meanColour, expectColour, numMean, plotPredicted, plotFitted, ignorePlanners)
    %rawPlannerData is :  numPlanners x {iter_number,cost} x numTrials x datapoints
    
    %Bah
    %errorThreshold = 10^-15; %Don't need this anymore, defined in terms of eps
    vErrTick = 100;
    labelFontSize = 14;
    legendFontSize = 12;
    markerType = 'o';
    markerSize = 5;
    numTrials = size(rawPlannerData,3);
    
    if size(rawPlannerData,1) ~= size(plannerNames,1)
        error('ASRL:plotConvergences:BadData', 'Number of names does not match number of planners in data');
    end

    %Preallocate:
    rateHandles = nan(size(rawPlannerData,1),1);
    diffHandles = nan(size(rawPlannerData,1),1);
    plotPlanners = true(size(rawPlannerData,1),1);
    
    for p = 1:size(rawPlannerData,1)
        rateHandles(p) = figure;
        hold on;

        skipPlanner = false;
        for j = 1:size(ignorePlanners,1)
            skipPlanner = skipPlanner || strcmp(plannerNames(p), ignorePlanners(j));
        end
        
        plotPlanners(p) = ~skipPlanner;

        %Get the iterations and costs: 
        thisIters = squeeze(rawPlannerData(p,1,:,:)); %(numTrials x datapoints)
        thisCost = squeeze(rawPlannerData(p,2,:,:)); %(numTrials x datapoints)
        
        %Calculate this error, accounting for numerical accuracy
        thisError = thisCost - minimumCost;
        thisError(thisError <= eps(minimumCost)) = 0.0;
        
        minError = min(min(thisError(thisError~=0)));
        maxError = max(max(thisError(thisError~=0)));
        
        %Plot:
        if plotPlanners(p) == true
            %Plot the trials as a thin mess of things:
            for j = 1:numTrials
                trialHandl = plot(thisIters(j,:), thisError(j,:));

                set(trialHandl, 'Color', trialColour);
                set(trialHandl, 'LineStyle', '-');
            end
            
            %Calculate the mean error, accounting for numerical accuracy
            meanCost = mean(thisCost(:,:));
            meanError = meanCost - minimumCost;
            meanError(meanError <= eps(minimumCost)) = 0.0;
            
            minError = min([minError min(meanError(meanError~=0.0))]);
            maxError = max([maxError max(meanError(meanError~=0.0))]);
            %(Plot last to place on top)
            
            %Get the first and last iteration
            firstIdx = find(isfinite(meanError), 1);
            firstIter = thisIters(1,firstIdx);
            lastIdx = find(meanError>0.0, 1, 'last');
            lastIter = thisIters(1,lastIdx);
            
            %Plot the expected value:
            if (plotPredicted)
                if strcmp(plannerNames(p), 'Informed RRT*')

                    %Set the initial cost
                    theoryCost = nan(1, lastIter - firstIter + 1,1);
                    theoryCost(1) = thisCost(1,firstIdx);

                    %Iterate the expected values:
                    for j = 2:size(theoryCost,2)
                        theoryCost(1,j) = (dimension*theoryCost(1,j-1)^2 + minimumCost^2)/((dimension+1)*theoryCost(1,j-1));
                    end

                    %Calculate the theoretical error, accounting for numerical accuracy
                    theoryError = theoryCost - minimumCost;
                    theoryError(theoryError <= eps(dimension+1)) = 0.0; %Dimension+1 as that's the largest number in the E[c] equation

                    minError = min([minError min(theoryError(theoryError~=0.0))]);
                    maxError = max([maxError max(theoryError(theoryError~=0.0))]);

                    %Plot the theory:
                    expectHandl = plot(firstIter:lastIter, theoryError);

                    set(expectHandl, 'Color', expectColour);
                    set(expectHandl, 'LineStyle', '-');
                    set(expectHandl, 'LineWidth', 3);
                    %set(expectHandl, 'Marker', markerType);
                    %set(expectHandl, 'MarkerSize', markerSize);

                else
                    error('Only Informed RRT*''s theoretical expected cost is currently implemented.');
                end
            end
            
            %Plot the mean:
            meanSpacing = ceil((lastIdx - firstIdx)/numMean);
            meanHandl = plot(thisIters(1,firstIdx:meanSpacing:lastIdx), meanError(1,firstIdx:meanSpacing:lastIdx));

            set(meanHandl, 'Color', meanColour);
            set(meanHandl, 'LineStyle', 'none');
            set(meanHandl, 'LineWidth', 3);
            set(meanHandl, 'Marker', markerType);
            set(meanHandl, 'MarkerSize', markerSize);
            
            %Plot a least-squares line to the mean data:
            if (plotFitted)
                %We assume the line is of the form: a*e^bx,
                expFunc = @(B,x)  B(1).*exp(B(2).*(x));
                
                %Calculate the (a,b) values where x has been shifted to start at 0 and y has been shifted up by 100:
                [beta(1), beta(2)] = fitExpCurve(0:(lastIter-firstIter), meanError(1,firstIdx:lastIdx), false);
                
                %Calculate the (a,b) values where x has been shifted to start at 0 and y has been shifted up by 100:
                %beta = nlinfit(0:(lastIter-firstIter), meanError(firstIdx:lastIdx), expFunc, [meanError(firstIdx) -1], statset('FunValCheck', 'off', 'Robust', 'on'));
                
                trendHandl = plot(firstIter:lastIter, expFunc(beta, 0:(lastIter-firstIter)));
            
                set(trendHandl, 'Color', meanColour);
                set(trendHandl, 'LineStyle', '--');
                set(trendHandl, 'LineWidth', 3);
            end
            
            %Configure the experimental plot:
            grid on;
            box off;
            set(gca, 'YScale','log');
            set(gca, 'YMinorGrid','on')

            %Set the x limit
            xlim([0.999*firstIter 1.001*max(max(thisIters(thisError~=0)))]);
            ylim([1e-15 3]);

            title(['R$^{' num2str(dimension) '}$ convergence'], 'FontSize', labelFontSize, 'Interpreter', 'latex');

            %Set the y axes
            ylabel('Logarithmic error', 'FontSize', labelFontSize, 'Interpreter', 'latex');

            %Set the x axis
            xlabel('Iteration', 'FontSize', labelFontSize, 'Interpreter', 'latex');
                    
            %Set the ratio
            pbaspect([3 1 1]);

            if (plotPredicted && plotFitted)
                lHandle = legend([trialHandl, meanHandl, trendHandl, expectHandl], {'Trials' 'Mean', 'Regression', 'Predicted'});
            elseif (plotPredicted && ~plotFitted)
                lHandle = legend([trialHandl, meanHandl, expectHandl], {'Trials' 'Mean' 'Predicted'});
            elseif (~plotPredicted && plotFitted)
                lHandle = legend([trialHandl, meanHandl, trendHandl], {'Trials' 'Mean', 'Regression'});
            else
                lHandle = legend([trialHandl, meanHandl], {'Trials' 'Mean'});
            end

            %Set the legend font, basic position, etc
            set(lHandle, 'FontSize', legendFontSize);
            set(lHandle, 'Interpreter', 'latex');
            set(lHandle, 'Orientation', 'Vertical');
            set(lHandle, 'Location', 'SouthOutside');

            
            
            
            %Plot the error plot if appropriate
            if (plotPredicted)
                if strcmp(plannerNames(p), 'Informed RRT*')
                    %Create the error handle
                    diffHandles(p) = figure;
                    
                    %Plot the difference between theory and mean. Skipping
                    %the first value which is 0 by definition.
                    errorHandl = plot((firstIter+1):lastIter, 100*abs((meanError(1,(firstIdx+1):lastIdx) - theoryError(1,2:end))./meanError(1,(firstIdx+1):lastIdx)));

                    set(errorHandl, 'Color', 'r');
                    set(errorHandl, 'LineStyle', '-');
                    set(errorHandl, 'LineWidth', 3);
                    
                    %Configure the plot:
                    grid on;
                    box off;
                    set(gca, 'YMinorGrid','on')

                    %Set the x limit
                    xlim([0.999*firstIter 1.001*max(max(thisIters(thisError~=0)))]);
                    ylim([0 100]);

                    title(['R$^{' num2str(dimension) '}$ prediction error'], 'FontSize', labelFontSize, 'Interpreter', 'latex');

                    %Set the y axes
                    ylabel('Relative prediction error', 'FontSize', labelFontSize, 'Interpreter', 'latex');

                    %Set the x axis
                    xlabel('Iteration', 'FontSize', labelFontSize, 'Interpreter', 'latex');
                    
                    %Set the ratio
                    pbaspect([6 1 1]);

                    lHandle = legend(errorHandl, 'Prediction Error');

                    %Set the legend font, basic position, etc
                    set(lHandle, 'FontSize', legendFontSize);
                    set(lHandle, 'Interpreter', 'latex');
                    set(lHandle, 'Orientation', 'Vertical');
                    set(lHandle, 'Location', 'SouthOutside');
                end
            end

        else
            %Announce
            fprintf('Skipped plotting %s.\n', plannerNames{p,1});
        end
    end
end
