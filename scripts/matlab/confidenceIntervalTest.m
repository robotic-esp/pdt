function out = confidenceIntervalTest(n, i, j)
    %Calculates the probability that the median the true population lies
    %within the open interval (i, j)
    
    if nargin < 3
        j = n - i + 1;
    end
    
%     %See: http://probabilityandstats.wordpress.com/2010/02/22/confidence-intervals-for-percentiles/
%     out = 0.0;
%     for k = i:(j-1)
%        out = out + (factorial(n)/(factorial(k)*factorial(n - k)))*(0.5^k)*(0.5^(n - k));
%     end
%     fprintf('CI (%d, %d): %6.6f\n', i, j, out);
    
%     %See: https://onlinecourses.science.psu.edu/stat414/node/316
%     if (j == n - i + 1)
%         out = 1 - 2*binocdf(i - 1,n,0.5);
%         fprintf('CI (%d, %d): %6.6f\n', i, n - i + 1, out);
%     end

    %If we undo the symmetry assumption from the second approach above, we get
    out = 1 - binocdf(i - 1,n,0.5) - binocdf(n - j,n,0.5);
%     fprintf('CI (%d, %d): %6.6f\n', i, j, out);
    
end