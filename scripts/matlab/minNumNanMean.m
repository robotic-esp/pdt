function rval = minNumNanMean(Y,dim,Nmin, Nmax)
    %returns a nanmean for the part of Y where there are [Nmin, Nmax)
    %non-nan values
    
    if ~ismatrix(Y)
        error('I dont want to think about this')
    end
    
    rvalSize = size(Y);
    rvalSize(dim) = 1;
    
    rval = nan(rvalSize);
    
    meanIdx = sum(~isnan(Y),dim) >= Nmin & sum(~isnan(Y),dim) < Nmax;
    
    repmatSize = size(Y);
    repmatSize(:) = 1;
    repmatSize(dim) = size(Y,dim);
    
    reshapeSize = size(Y);
    if dim == 1
        reshapeSize(2) = sum(meanIdx);
    elseif dim == 2
        reshapeSize(1) = sum(meanIdx);
    else
        error('fuck off');
    end
    
	rval(meanIdx) = nanmean(reshape(Y(repmat(meanIdx,repmatSize)), reshapeSize),dim);
end