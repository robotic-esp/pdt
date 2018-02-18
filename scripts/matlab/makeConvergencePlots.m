%Process multiple files into plots
function makeConvergencePlots
    %Updated Matlab colours
    % colours = permute(get(gca, 'colororder'), [1 3 2]);
    % colours_resize = imresize(colours, 50.0, 'nearest');
    % imshow(colours_resize);
    % get(gca, 'colororder')
    y = [0.9290 0.6940 0.1250]; %3
    m = [0.4940 0.1840 0.5560]; %4
    c = [0.3010 0.7450 0.9330]; %6
    r = [0.6350 0.0780 0.1840]; %7
    g = [0.4660 0.6740 0.1880]; %5
    b = [0 0.4470 0.7410]; %1
    o = [0.8500 0.3250 0.0980]; %2
    w = [1 1 1];
	k = [0 0 0];

%     %Colours (https://kuler.adobe.com):
%     desat = 0.9;
%     y = desat*[1 1 0]; %[127 127 0]/255; %[1 1 0];
%     m = desat*[1 0 1]; %[127 0 127]/255; %[1 0 1];
%     c = desat*[0 1 1]; %[0 127 127]/255; %[0 1 1];
%     r = desat*[1 0 0]; %[127 0 0]/255; %[1 0 0];
%     g = desat*[0 1 0]; %[0 127 0]/255; %[0 1 0];
%     b = desat*[0 0 1]; %[0 0 127]/255; %[0 0 1];
%     w = [1 1 1];
%     k = [0 0 0];
    
    %The minimum
    minCost = 1.0;
    numMean = 35;

    %The filesnames (ommit the data/ and the .csv), the dimension, the number of experiments, the spacing for the mean-cost plotting, whether to plot predicted, whether to plot fitted
    filenames = {
                 %%% Steer: Infinite. Rewire: Infinite
                 'ConvergeR2S3599049271SteerinfRewireinf', 2, 10000, true, true;
                 'ConvergeR4S4138350479SteerinfRewireinf', 4, 10000, true, true;
                 'ConvergeR8S4235719018SteerinfRewireinf', 8, 10000, true, true;
                 
%                  %%% Steer: Finite. Rewire: Infinite
%                 'ConvergeR2S505806321Steer0.4Rewireinf', 2, 10000, true, true;
%                  'ConvergeR4S673083786Steer0.4Rewireinf', 4, 10000, true, true;
%                  'ConvergeR8S201999010Steer0.4Rewireinf', 8, 10000, true, true;
%                  
%                  %%% Steer: Infinite. Rewire: Finite
%                  'ConvergeR2S3380876990SteerinfRewire1.1', 2, 10000, true, false;
%                  'ConvergeR4S2607726372SteerinfRewire1.1', 4, 10000, true, false;
%                  'ConvergeR8S1735541730SteerinfRewire1.1', 8, 10000, true, false;
                 
                 %%% Steer: Finite. Rewire: Finite
%                  'ConvergeR2S3687473251Steer0.4Rewire1.1', 2, 10000, false, false;
%                  'ConvergeR4S3223027747Steer0.4Rewire1.1', 4, 10000, false, false;
%                  'ConvergeR8S105787506Steer0.4Rewire1.1', 8, 10000, false, false
                 };

    %The planner names
    planners = {'Informed RRT*'};

    ignorePlanners = {};

    for i = 1:size(filenames,1)
        data = processIterCostData(['data/' filenames{i,1} '.csv'], filenames{i,3}, minCost);
        [rateHandl, errorHandl] = plotConvergences(data, planners, filenames{i,2}, minCost, c, b, k, numMean, filenames{i,4}, filenames{i,5}, ignorePlanners);

        for j = 1:size(planners,1)
            saveEpsToPdf(rateHandl(j), ['data/' filenames{i,1} '_Planner' num2str(j) '.pdf']);
            close(rateHandl(j));
            if isgraphics(errorHandl(j),'Figure')
               saveEpsToPdf(errorHandl(j), ['data/' filenames{i,1} '_Planner' num2str(j) '_Error.pdf']);
               close(errorHandl(j));
            end
        end
    end
end