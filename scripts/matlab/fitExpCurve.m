function [a,b] = fitExpCurve(x, y, leastSquares)
    %Fits y = a*exp(b*x) using: http://mathworld.wolfram.com/LeastSquaresFittingExponential.html
    
    if nargin < 3
        leastSquares = true;
    end

    if ~isvector(x)
        error('Expect x to be Nx1 or 1xN');
    elseif ~isvector(y) || size(x,1) ~= size(y,1) || size(x,2) ~= size(y,2)
        error('Expect y to be Nx1 or 1xN');
    end
    
    if leastSquares
        %Least-squares fit
        A = sum(x.^2.*y)*sum(y.*log(y)) - sum(x.*y)*sum(x.*y.*log(y));
        B = sum(y)*sum(x.*y.*log(y)) - sum(x.*y)*sum(y.*log(y));
        denom = sum(y)*sum(x.^2.*y) - sum(x.*y)^2;
    else
        %Best-fit
        A = sum(log(y))*sum(x.^2) - sum(x)*sum(x.*log(y));
        B = length(x)*sum(x.*log(y)) - sum(x)*sum(log(y));
        denom = length(x)*sum(x.^2) - sum(x)^2;
    end
        
    a = exp(A/denom);
    b = B/denom;
end