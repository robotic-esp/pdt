function confidenceIntervalTest(n, i, j)
    %Calculates the resulting distribution-free confidence interval for the
    %median of n samples if you choose cut off values at i and j.
    %See: http://probabilityandstats.wordpress.com/2010/02/22/confidence-intervals-for-percentiles/
    out = 0.0;

    for k = i:j
        out = out + (factorial(n)/(factorial(k)*factorial(n - k)))*(0.5^k)*(0.5^(n - k));
    end

    fprintf('CI: %6.6f\n', out);
end