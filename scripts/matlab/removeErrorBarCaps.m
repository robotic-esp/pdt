function removeErrorBarCaps(errHandle)
    if ~strcmp(version, '8.6.0.267246 (R2015b)')
        error ('For versions >= R2016b see: https://uk.mathworks.com/matlabcentral/answers/100333-how-do-i-change-the-width-of-the-horizontal-lines-at-top-and-bottom-of-error-bars-in-my-errorbar-plo#answer_109681');
    end
    
%     X = errHandle.XData;
% 
%     % Multiply the size of the cap-lines
%     mult = 0;
% 
%     % hidden property/handle
%     b = get(errHandle, 'Bar');
% 
%     % populate b's properties (required)
%     drawnow;
%     
%     vd = get(b, 'VertexData');
% 
%     % number of error bars
%     N = numel(X);
% 
%     % assumes equal length on all
%     capLength = vd(1,2*N+2,1) - vd(1,1,1);    
%     newLength = capLength * mult;
%     leftInds = N*2+1:2:N*6;
%     rightInds = N*2+2:2:N*6;
%     vd(1,leftInds,1) = [X-newLength, X-newLength];
%     vd(1,rightInds,1) = [X+newLength, X+newLength];
%     set(b, 'VertexData', vd);
end