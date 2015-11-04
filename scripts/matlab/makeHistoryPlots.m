%Process multiple files into plots
function makeHistoryPlots(R)
    %Interp spacing:
    interpTime = 0.001;

    %Colours (https://kuler.adobe.com):
    desat = 0.9;
    y = desat*[1 1 0]; %[127 127 0]/255; %[1 1 0];
    m = desat*[1 0 1]; %[127 0 127]/255; %[1 0 1];
    c = desat*[0 1 1]; %[0 127 127]/255; %[0 1 1];
    r = desat*[1 0 0]; %[127 0 0]/255; %[1 0 0];
    g = desat*[0 1 0]; %[0 127 0]/255; %[0 1 0];
    b = desat*[0 0 1]; %[0 0 127]/255; %[0 0 1];
    w = [1 1 1];
	k = [0 0 0];
    
    %Plot time:
    if (R == 0)
        %Interp spacing:
        interpTime = 0.001;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 25;
    
        plotTime = 300;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25; 

        %The filesnames ommit the data/ and the .csv.
        filenames = {'R8S1664403761RegularRects'
                     };

        figures = {};

        %The planner names (log and plot) and colours:
        planners = { 'RRTstar_Trio',                'RRT* trio',                b;
                     'Informed_RRTstar',            'Informed RRT*',            g};
       
       ignorePlanners = {};
    elseif (R == 2)
        %Interp spacing:
        interpTime = 0.001;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 3;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25; 

        %The filesnames ommit the data/ and the .csv.
        filenames = {'R2S431254373Random';
                     'R2S1033176963Random';
                     'R2S1638379583Random';
                     'R2S1674882576Random';
                     'R2S2238807225Random';
                     'R2S2278048979Random';
                     'R2S2878783453Random';
                     'R2S3483583633Random';
                     'R2S4084286803Random';
                     'R2S4120957129Random'
                     };

        figures = {};

        %The planner names (log and plot) and colours:
        planners = { 'RRTstar',                     'RRT*',                     k;
                     'RRTstar_Prune',               'RRT* w/ pruning',          r;
                     'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
                     'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
                     'RRTstar_Trio',                'RRT* trio',                b;
                     'Informed_RRTstar',            'Informed RRT*',            g};
       
       ignorePlanners = {};
    elseif (R == 4)
        %Interp spacing:
        interpTime = 0.001;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 30;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25; 

        %The filesnames ommit the data/ and the .csv.
        filenames = {'R4S23022343Random';
                     'R4S70368211Random';
                     'R4S866562609Random';
                     'R4S943802410Random';
                     'R4S1739490818Random';
                     'R4S1787630856Random';
                     'R4S2583261783Random';
                     'R4S2630985384Random';
                     'R4S3474382323Random';
                     'R4S3521488590Random'
                    };

        figures = {};

        %The planner names (log and plot) and colours:
        planners = { 'RRTstar',                     'RRT*',                     k;
                     'RRTstar_Prune',               'RRT* w/ pruning',          r;
                     'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
                     'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
                     'RRTstar_Trio',                'RRT* trio',                b;
                     'Informed_RRTstar',            'Informed RRT*',            g};
       
       ignorePlanners = {};
    elseif (R == 8)
        %Interp spacing:
        interpTime = 0.001;
    
        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 150;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25; 

        %The filesnames ommit the data/ and the .csv.
        filenames = {'R8S3273344990Random';
                     'R8S3349575148Random';
                     'R8S3426995241Random'
                    };

        figures = {};

        %The planner names (log and plot) and colours:
        planners = { 'RRTstar',                     'RRT*',                     k;
                     'RRTstar_Prune',               'RRT* w/ pruning',          r;
                     'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
                     'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
                     'RRTstar_Trio',                'RRT* trio',                b;
                     'Informed_RRTstar',            'Informed RRT*',            g};
       
       ignorePlanners = {};
    else
        error('Unhandled argument');
    end

    succRates = nan(0,0,0);
    for i = 1:size(filenames,1)
        %Check the file:
        scanFiles(['data/' filenames{i} '.csv'], planners(:,1), true, true);
        
        data = processSimHistory(['data/' filenames{i} '.csv'], numExp, plotTime, interpTime);
        [succHandl, histHandl, succRates(:,:,i)] = plotSimHistory(data, planners(:,2), planners(:,3), ignorePlanners, useMedian, unsolvedAsNan, yLimMargin);

        saveEpsToPdf(succHandl, ['data/' filenames{i} '_success.pdf']);
        close(succHandl)

        saveEpsToPdf(histHandl, ['data/' filenames{i} '_hist.pdf']);
        close(histHandl)
    end

    if ~isempty(figures)
        cd data/plots/
        for i = 1:size(figures,1)
            figure;
            eval(figures{i});
            saveEpsToPdf(1, ['../' figures{i} '.pdf']);
            close;
        end
        cd ../../
    end
end