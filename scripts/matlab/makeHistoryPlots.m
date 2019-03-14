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


    %Plot time:
    if (R == 0.2)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;
    
        %The run time
        plotTime = 1.0;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {
            'R8S18439072146874402233WallGap'
%             'R8S18439071351109370200RegularRects'
%             'R8S18439071143733709969RandRect'
%             'R4S18439071184667532744RandRect'
%             'R4S18439071184930502469RandRect'
%             'R4S18439071185187944460RandRect'
%             'R4S18439071185450992270RandRect'
%             'R4S18439071185709980611RandRect'
%             'R4S18439071185974167306RandRect'
%             'R4S18439071186242979955RandRect'
        };

        figures = {};

        planners = {
                    'BITstar100',            'BIT*',             b, '-';
%                     'BITstarRegression100', 'BIT* (reg)',       c, '-';
                    'ABITstar100',           'SBIT*',            g, '-';
                    'RRTConnect',            'RRTConnect',       k, '-';
                    'LBTRRT',                'LBTRRT',           p, '-';
                    'RRTstar',               'RRT*',             r, '-';
                    'RRTsharp',              'RRT$^{\#}$',       o, '-';
                    };

        ignorePlanners = {'DELETEME'};   
    elseif (R == 0.4)
        %Interp spacing (us):
        interpTime = 1000*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;
    
        %The run time
        plotTime = 10;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R8S18439070324758262881RandRect'};

        figures = {};

        %The planner names (log and plot) and colours:
        planners = {
                    'BITstar1000',           'BIT*',             b, '-';
%                     'BITstarRegression300', 'BIT* (reg)',       c, '-';
                    'ABITstar1000',          'ABIT*',            g, '-';
                    'RRTConnect',           'RRTConnect',       k, '-'
                    };

        ignorePlanners = {'DELETEME'};
    elseif (R == 0.8)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 1000;
    
        %The run time
        plotTime = 5;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R4S18439067383613633415RegularRects_without_pruning'};

        figures = {};

        %The planner names (log and plot) and colours:
        planners = {'ABITstar100',        'ABIT*',         b, '-'};

        ignorePlanners = {'DELETEME'};

    elseif (R == 2)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;
    
        %The run time
        plotTime = 1.0;

        %The filesnames. Ommit the data/ and the .csv.
        planners = {'BITstar100',           'BIT*',             b, '-';
                    'BITstarRegression100', 'BIT* (reg)',       y, '-';
                    'ABITstar100',          'ABIT*',            g, '-'};
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
        
    elseif (R == 2.1)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;
    
        %The run time    
        plotTime = 1.0;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R2S18439041981077254091DblEncl';
                     'R2S18439042010624099317RegularRects'};

        figures = {};

        %The planner names (log and plot) and colours:
        planners = {'BITstar5',           'BIT* (5)'      r, '-';
                    'BITstar10',          'BIT* (10)'     o, '-';
                    'BITstar50',          'BIT* (50)'     y, '-';
                    'BITstar100',         'BIT* (100)'    g, '-';
                    'BITstar500',         'BIT* (500)'    c, '-';
                    'BITstar1000',        'BIT* (1000)'   b, '-';
                    'BITstar5000',        'BIT* (5000)'   p, '-';};

        ignorePlanners = {'DELETEME'};   
    elseif (R == 4)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;

        %The run time
        plotTime = 10;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R4S18439042524097082794RandRect';
                     'R4S18439042529970181147RandRect';
                     'R4S18439042535910568965RandRect';
                     'R4S18439042541803537546RandRect';
                     'R4S18439042547670616412RandRect';
                     'R4S18439042553637184069RandRect';
                     'R4S18439042559574700751RandRect';
                     'R4S18439042565496571016RandRect';
                     'R4S18439042571460205512RandRect';
                     'R4S18439042577288193012RandRect'};

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
        
    elseif (R == 4.1)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;

        %The run time    
        plotTime = 10;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R4S18439041981805099620DblEncl';
                     'R4S18439042011367949599RegularRects'};

        figures = {};

        %The planner names (log and plot) and colours:
        planners = {'BITstar5',           'BIT* (5)'      r, '-';
                    'BITstar10',          'BIT* (10)'     o, '-';
                    'BITstar50',          'BIT* (50)'     y, '-';
                    'BITstar100',         'BIT* (100)'    g, '-';
                    'BITstar500',         'BIT* (500)'    c, '-';
                    'BITstar1000',        'BIT* (1000)'   b, '-';
                    'BITstar5000',        'BIT* (5000)'   p, '-';};


        ignorePlanners = {'DELETEME'};
        
    elseif (R == 8)
        %Interp spacing (us):
        interpTime = 100*1e-6;

        %Number of experiments in each file:
        numExp = 100;

        %The run time
        plotTime = 30;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R8S18439042822012678066RandRect';
                     'R8S18439042840587660934RandRect';
                     'R8S18439042859586030721RandRect'};

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
        
    elseif (R == 8.1)
        %Interp spacing (us):
        interpTime = 100*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;
    
        %The run time
        plotTime = 30;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R8S18439041988907160763DblEncl';
                     'R8S18439042018432131306RegularRects'};

        figures = {};

        %The planner names (log and plot) and colours:
        planners = {'BITstar5',           'BIT* (5)'      r, '-';
                    'BITstar10',          'BIT* (10)'     o, '-';
                    'BITstar50',          'BIT* (50)'     y, '-';
                    'BITstar100',         'BIT* (100)'    g, '-';
                    'BITstar500',         'BIT* (500)'    c, '-';
                    'BITstar1000',        'BIT* (1000)'   b, '-';
                    'BITstar5000',        'BIT* (5000)'   p, '-';};
                
        ignorePlanners = {'DELETEME'};
        
    elseif (R == 16)
        %Interp spacing (us):
        interpTime = 1000*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;

        %The run time
        plotTime = 100;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R16S18439043050149215929RandRect';
                     'R16S18439043113440834383RandRect';
                     'R16S18439043177570894870RandRect';
                     'R16S18439043242343379133RandRect';
                     'R16S18439043305624313035RandRect';
                     'R16S18439043370829349683RandRect';
                     'R16S18439043435870313265RandRect'};

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

    elseif (R == 16.1000)
        %Interp spacing (us):
        interpTime = 2000*1e-6;
        
        %Number of experiments in each file:
        numExp = 100;

        %The run time
        plotTime = 1000;

        %The filesnames. Ommit the data/ and the .csv.
        filenames = {'R16S18439043871333544029DblEncl'};
        
        %Force infinte CIs
        plotInfiniteCIs = true;

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

