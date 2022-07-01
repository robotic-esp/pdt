function [lowIdx, highIdx, confidence] = computeConfidenceInterval(n, p)
    % Computes the interval for the median of n samples given a confidence
    % probability p.

    startSearchIdx = 0;
    endSearchIdx = n;
    if n <= 100 % just brute force it.
        startSearchIdx = 0;
        endSearchIdx = n;
    elseif n < 1000 % be a little less conservative.
        startSearchIdx = ceil(n/3);
        endSearchIdx = ceil(2*n/3);
    elseif n < 5000 % be a little less conservative still.
        startSearchIdx = ceil(6*n/15);
        endSearchIdx = ceil(9*n/15);
    elseif n < 20000 % be a little less conservative still.
        startSearchIdx = ceil(14*n/31);
        endSearchIdx = ceil(17*n/31);
    elseif n < 100000 % be a little less conservative still.
        startSearchIdx = ceil(35*n/71);
        endSearchIdx = ceil(36*n/71);
    elseif n < 1000000 % be a little less conservative still.
        startSearchIdx = ceil(60*n/121);
        endSearchIdx = ceil(61*n/121);
    elseif n == 1000000 % be a little less conservative still.
        startSearchIdx = ceil(170*n/341);
        endSearchIdx = ceil(171*n/341);
    else
        error('Provide bounds, this will take forever.');
    end

    fprintf('lowest considered idx: %d, highest considered idx: %d', startSearchIdx, endSearchIdx);

    bestInterval = [startSearchIdx, endSearchIdx];
    bestPercentage = 1 - binocdf(startSearchIdx - 1,n,0.5) - binocdf(n - endSearchIdx,n,0.5);
    if bestPercentage < p
        error("Search is not conservative enough. Lower the startSearchIdx and increase the endSearchIdx.");
    end

    for i = startSearchIdx:1:floor(n/2)
        for j = ceil(n/2):1:endSearchIdx
            intervalPercentage = 1 - binocdf(i - 1,n,0.5) - binocdf(n - j,n,0.5);
            if intervalPercentage - p >= 0 && j - i < bestInterval(2) - bestInterval(1) 
                bestInterval = [i, j];
                bestPercentage = intervalPercentage;
                break; % We can safely brake here, any larger j will give a larger interval and higher percentage.
            end
        end
    end
    lowIdx = bestInterval(1);
    highIdx = bestInterval(2);
    confidence = bestPercentage;
end
