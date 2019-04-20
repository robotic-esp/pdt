%Process multiple files into plots
function makeHistoryPlots(R)
    %Interp spacing:
    interpTime = 0.001;
    
    %Updated Matlab colours
    % colours = permute(get(gca, 'colororder'), [1 3 2]);
    % colours_resize = imresize(colours, 50.0, 'nearest');
    % imshow(colours_resize);
    % get(gca, 'colororder')
    y = [0.9290 0.6940 0.1250]; %3
    p = [0.4940 0.1840 0.5560]; %4
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


    %Configure the common plotting
    %Use medians
    useMedian = true;
    %Mark unsolved as nan (instead of inf)
    unsolvedAsNan = false;
    %The relative margin above the medians that we're willing to plot...
    yLimMargin = 0.25;
    %Plot planners without any results
    plotFailures = true;
    %Whether to plot CIs with an infinite range:
    plotInfiniteCIs = false;

    %Interp spacing (us):
    interpTime = 1 / 10000.0;

    %Number of experiments in each file:
    numExp = 100;

    %The run time
    plotTime = 1;

    %The filesnames. Ommit the data/ and the .csv.
    filenames = {
      '2019-04-18_16-03-54_RandomRectangles4D'
    };

    figures = {};

    planners = {
                'SBITstar',               'BIT*',             b, '-';
%                 'BITstarRegression100',  'BIT* (reg)',       c, '-';
                'BITstar',              'SBIT*',            g, '-';
%                 'RRTConnect',            'RRTConnect',       k, '-';
%                 'LBTRRT',                'LBTRRT',           p, '-';
                'RRTstar',               'RRT*',             r, '-';
                'RRTsharp',              'RRT$^{\#}$',       o, '-';
                };

    ignorePlanners = {};
  
    succRates = nan(0,0,0);
    for i = 1:size(filenames,1)
        %Check the file:
        scanFiles(['data/' filenames{i} '.csv'], planners(:,1), true, true);
        
        data = processSimHistory(['data/' filenames{i} '.csv'], numExp, plotTime, interpTime);
        [succHandl, histHandl, succRates(:,:,i)] = plotSimHistory(data, planners(:,2), planners(:,3), planners(:,4), plotTime, ignorePlanners, useMedian, unsolvedAsNan, plotFailures, plotInfiniteCIs, yLimMargin);
%
%         saveEpsToPdf(succHandl, ['data/' filenames{i} '_success.pdf'], true);
%         close(succHandl)
%
%         saveEpsToPdf(histHandl, ['data/' filenames{i} '_hist.pdf'], true);
%         close(histHandl)
    end

    if ~isempty(figures)
        cd data/plots/
        for i = 1:size(figures,1)
            figure;
            eval(figures{i});
            saveEpsToPdf(1, ['../' figures{i} '.pdf'], true);
            close;
        end
        cd ../../
    end
end

