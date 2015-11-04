function rval = minNumNanMedian(Y,dim,Nmin, Nmax)
    %returns a nanmedian for the part of Y where there are [Nmin, Nmax)
    %non-nan values
    
    if ~ismatrix(Y)
        error('I dont want to think about this')
    end
    
    rvalSize = size(Y);
    rvalSize(dim) = 1;
    
    rval = nan(rvalSize);
    
    medianIdx = sum(~isnan(Y),dim) >= Nmin & sum(~isnan(Y),dim) < Nmax;
    
    repmatSize = size(Y);
    repmatSize(:) = 1;
    repmatSize(dim) = size(Y,dim);
    
    reshapeSize = size(Y);
    if dim == 1
        reshapeSize(2) = sum(medianIdx);
    elseif dim == 2
        reshapeSize(1) = sum(medianIdx);
    else
        error('fuck off');
    end
    
	rval(medianIdx) = nanmedian(reshape(Y(repmat(medianIdx,repmatSize)), reshapeSize),dim);
end