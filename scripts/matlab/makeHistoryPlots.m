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


    %Plot time:
    if (R == 0)
        %Interp spacing (us):
        interpTime = 100*1e-6;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 3.0;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25;
        
        %Whether to plot CIs with an infinite range:
        plotInfiniteCIs = false;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {''};

        figures = {};


        %The planner names (log and plot) and colours:
        planners = {'RRTConnect',        'RRT-Connect',         k, '-';
                    'RRTsharp',          'RRT\#',               r, '-';
                    'RRTsharp1',         'RRT\#1',              p, '-';
                    'RRTsharp2',         'RRT\#2',              o, '-';
                    'RRTsharp3',         'RRT\#3',              y, '-';
                    'Informed_RRTstar',  'Informed RRT*',       c, '-';
                    'SORRTstar100',      'SORRT*',              b, '-';
                    'BITstar100',        'BIT*',                g, '-'};

        ignorePlanners = {'DELETEME'};

    elseif (R == 2)
        %Interp spacing (us):
        interpTime = 100*1e-6;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 3.0;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25;
        
        %Whether to plot CIs with an infinite range:
        plotInfiniteCIs = false;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R2S18439039726546219644DblEncl'};

        figures = {};


        %The planner names (log and plot) and colours:
        planners = {'RRTConnect',           'RRT-Connect'     k, '-';
                    'RRT',                  'RRT',            y, '-';
                    'RRTstar',              'RRT*',           r, '-';
                    'RRTsharp3',            'RRT\#',          o, '-';
                    'Informed_RRTstar',     'Informed RRT*',  c, '-';
                    'FMTstar100',           'FMT*',           p, '-';
                    'FMTstar1000',          'FMT*',           p, '-';
                    'FMTstar10000',         'FMT*',           p, '-';
                    'SORRTstar100',         'SORRT*',         b, '-';
                    'BITstar100',           'BIT*',           g, '-';};

        ignorePlanners = {'DELETEME'};   
    elseif (R == 4)
        %Interp spacing (us):
        interpTime = 100*1e-6;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 100;
    
        plotTime = 10;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25;
        
        %Whether to plot CIs with an infinite range:
        plotInfiniteCIs = false;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R4S18439039728190501169DblEncl'};

        figures = {};


        %The planner names (log and plot) and colours:
        planners = {'RRTConnect',           'RRT-Connect'     k, '-';
                    'RRT',                  'RRT',            y, '-';
                    'RRTstar',              'RRT*',           r, '-';
                    'RRTsharp3',            'RRT\#',          o, '-';
                    'Informed_RRTstar',     'Informed RRT*',  c, '-';
                    'FMTstar100',           'FMT*',           p, '-';
                    'FMTstar1000',          'FMT*',           p, '-';
                    'FMTstar10000',         'FMT*',           p, '-';
                    'SORRTstar100',         'SORRT*',         b, '-';
                    'BITstar100',           'BIT*',           g, '-';};

        ignorePlanners = {'DELETEME'};
        
    elseif (R == 8)
        %Interp spacing (us):
        interpTime = 100*1e-6;

        useMedian = true;
        unsolvedAsNan = false;
        
        %Number of experiments in each file:
        numExp = 250;
    
        plotTime = 10;
        
        %The relative margin above the medians that we're willing to plot...
        yLimMargin = 0.25;
        
        %Whether to plot CIs with an infinite range:
        plotInfiniteCIs = false;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R8S18439039522079872659DblEncl'};

        figures = {};


        %The planner names (log and plot) and colours:
        planners = {'RRTConnect',           'RRT-Connect'     k, '-';
                    'RRT',                  'RRT',            y, '-';
                    'RRTstar',              'RRT*',           r, '-';
                    'RRTsharp3',            'RRT\#',          o, '-';
                    'Informed_RRTstar',     'Informed RRT*',  c, '-';
                    'FMTstar100',           'FMT*',           p, '-';
                    'FMTstar1000',          'FMT*',           p, '-';
                    'FMTstar10000',         'FMT*',           p, '-';
                    'SORRTstar100',         'SORRT*',         b, '-';
                    'BITstar100',           'BIT*',           g, '-';};

        ignorePlanners = {'DELETEME'};
    else
        error('Unhandled argument');
    end

    succRates = nan(0,0,0);
    for i = 1:size(filenames,1)
        %Check the file:
        scanFiles(['data/' filenames{i} '.csv'], planners(:,1), true, true);
        
        data = processSimHistory(['data/' filenames{i} '.csv'], numExp, plotTime, interpTime);
        [succHandl, histHandl, succRates(:,:,i)] = plotSimHistory(data, planners(:,2), planners(:,3), planners(:,4), ignorePlanners, useMedian, unsolvedAsNan, plotInfiniteCIs, yLimMargin);

        saveEpsToPdf(succHandl, ['data/' filenames{i} '_success.pdf'], true);
        close(succHandl)

        saveEpsToPdf(histHandl, ['data/' filenames{i} '_hist.pdf'], true);
        close(histHandl)
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













%Old run-configs
% %%%%%
%     if (R == 0.2)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 1;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R2S1916971778SpiralExperiment'
%                      };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = {'RRTConnect',              'RRTConnect',               k;
%                     'RRT',                     'RRT',                      y;
%                     'RRTstar',                 'RRT*',                     m;
%                     'Informed_RRTstar',        'Informed RRT*',            c;
%                     'SORRTstar100',            'SORRT*',                   b;
%                     'FMTstar100',              'FMT*',                     r;
%                     'FMTstar1000',             'FMT*',                     r;
%                     'FMTstar10000',            'FMT*',                     r;
%                     'BITstar100',              'BIT*',                     g};
% 
%        
%        ignorePlanners = {};
%     elseif (R == 0.8)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 50;
%     
%         plotTime = 1.0*60;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R8S2926895744RandRect'
%                      };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = {'RRTConnect',              'RRTConnect',               k;
%                     'Informed_RRTstar',        'Informed RRT*',            c;
%                     'SORRTstar100',            'SORRT*',                   b;
%                     'BITstar100',              'BIT*',                     g};
%        
%        ignorePlanners = {};
%     elseif (R == 2)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 3;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R2S1768893106RegularRects';
%                      'R2S69304096RandRect';
%                      'R2S1318982274RandRect';
%                      'R2S1392623785RandRect';
%                      'R2S1474871623RandRect';
%                      'R2S1559415480RandRect';
%                      'R2S2793489766RandRect';
%                      'R2S2879561549RandRect';
%                      'R2S2959607242RandRect';
%                      'R2S4210929957RandRect';
%                      'R2S4275578477RandRect'
%                      };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = {'RRTConnect',              'RRTConnect',               c;
%                     'RRT',                     'RRT',                      k;
%                     'RRTstar',                 'RRT*',                     m;
%                     'Informed_RRTstar',        'Informed RRT*',            b;
%                     'FMTstar100',              'FMT*',                     r;
%                     'FMTstar1000',             'FMT*',                     r;
%                     'FMTstar10000',            'FMT*',                     r;
%                     'BITstar100',              'BIT*',                     g};
%        
%        ignorePlanners = {};
%     elseif (R == 8)
%         %Interp spacing:
%         interpTime = 0.001;
%     
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 150;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R8S2931786282RegularRects';
%                      'R8S611884414RandRect';
%                      'R8S666381624RandRect';
%                      'R8S1005043852RandRect';
%                      'R8S16758865RandRect';
%                      'R8S2090761801RandRect';
%                      'R8S2563592664RandRect';
%                      'R8S2733150352RandRect';
%                      'R8S3119022076RandRect';
%                      'R8S3999221943RandRect';
%                      'R8S4243632470RandRect'
%                     };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = {'RRTConnect',              'RRTConnect',               c;
%                     'RRT',                     'RRT',                      k;
%                     'RRTstar',                 'RRT*',                     m;
%                     'Informed_RRTstar',        'Informed RRT*',            b;
%                     'FMTstar100',              'FMT*',                     r;
%                     'FMTstar1000',             'FMT*',                     r;
%                     'FMTstar10000',            'FMT*',                     r;
%                     'BITstar100',              'BIT*',                     g};
%        
%        ignorePlanners = {};
%     elseif (R == 16)
%         %Interp spacing:
%         interpTime = 0.001;
%     
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 300;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R16S704792960RegularRects';
% %                      'R16S530875199RandRect';
% %                      'R16S3720434557RandRect';
% %                      'R16S4153856203RandRect';
%                     };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = {'RRTConnect',              'RRTConnect',               c;
%                     'RRT',                     'RRT',                      k;
%                     'RRTstar',                 'RRT*',                     m;
%                     'Informed_RRTstar',        'Informed RRT*',            b;
%                     'FMTstar100',              'FMT*',                     r;
%                     'FMTstar1000',             'FMT*',                     r;
%                     'FMTstar10000',            'FMT*',                     r;
%                     'BITstar100',              'BIT*',                     g};
%        
%        ignorePlanners = {};
%     elseif (R == 2i)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 3;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R2S4008035615RegularRects'
%                     };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = { 'RRTstar',                     'RRT*',                     k;
%                      'RRTstar_Prune',               'RRT* w/ pruning',          r;
%                      'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
%                      'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
%                      'RRTstar_Trio',                'RRT* trio',                b;
%                      'Informed_RRTstar',            'Informed RRT*',            g};
%        
%        ignorePlanners = {};
%     elseif (R == 4i)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 30;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R4S1810018230RegularRects'
%                     };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = { 'RRTstar',                     'RRT*',                     k;
%                      'RRTstar_Prune',               'RRT* w/ pruning',          r;
%                      'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
%                      'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
%                      'RRTstar_Trio',                'RRT* trio',                b;
%                      'Informed_RRTstar',            'Informed RRT*',            g};
%        
%        ignorePlanners = {};
%     elseif (R == 8i)
%         %Interp spacing:
%         interpTime = 0.001;
% 
%         useMedian = true;
%         unsolvedAsNan = false;
%         
%         %Number of experiments in each file:
%         numExp = 100;
%     
%         plotTime = 150;
%         
%         %The relative margin above the medians that we're willing to plot...
%         yLimMargin = 0.25; 
% 
%         %The filesnames ommit the data/ and the .csv.
%         filenames = {'R8S633746805RegularRects'
%                     };
% 
%         figures = {};
% 
%         %The planner names (log and plot) and colours:
%         planners = { 'RRTstar',                     'RRT*',                     k;
%                      'RRTstar_Prune',               'RRT* w/ pruning',          r;
%                      'RRTstar_NewStateRejection',   'RRT* w/ new reject',       m;
%                      'RRTstar_SampleRejection',     'RRT* w/ sample reject',    c;
%                      'RRTstar_Trio',                'RRT* trio',                b;
%                      'Informed_RRTstar',            'Informed RRT*',            g};
%        
%        ignorePlanners = {};
% %%%%%