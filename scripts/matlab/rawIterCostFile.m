function [resultIter, resultCost] = rawIterCostFile(filename,minCost)
    %Start empty
    resultIter = 0;
    iterCell = {};
    costCell = {};
    
    %Open the file
    [fid theMessage] = fopen(filename);
	if fid < 0
		error('ASRL:rawIterCostFile:FileNotOpened', 'The file ''%s'' could not be opened because: %s',filename,theMessage);
	end
    
    delimiter = sprintf(',');
    skipColumns = 1;
    %If you wanted to skip rows, you'd need to just jump over processing an appropriate number of times
    
    %Iterate over the lines
    lineNum = 0;
    while 1
        %Increment our line number
        lineNum = lineNum + 2;
        
        %Get the lines
        iterString = fgetl(fid);
        costString = fgetl(fid);
    
        %Break cleanly if it's the end of file
        if ~ischar(iterString) && ~ischar(costString)
            break;
        elseif ~ischar(iterString) || ~ischar(costString)
            error('ASRL:rawIterCostFile:NumberOfLines', 'Unexpectedly found an odd number of lines in %s.', filename);
        end
        
        %Find the deliminators in the string
        iterDelimPos = strfind(iterString, delimiter);
        costDelimPos = strfind(costString, delimiter);
        
        %Check their labels
        if (strcmp( iterString(1:iterDelimPos(skipColumns)), costString(1:costDelimPos(skipColumns)) ) == false)
            error('ASRL:rawIterCostFile:BadData', 'The header of lines %d--%d do not match in %s.', lineNum-1, lineNum, filename);
        end
        
        %Crop the strings
        croppedIter = iterString(iterDelimPos(skipColumns):end);
        croppedCost = costString(costDelimPos(skipColumns):end);
        
        %Process the strings:
        [iterData,~,iterError,~] = sscanf(croppedIter, ',%d');
        [costData,~,costError,~] = sscanf(croppedCost, ',%f');
        
        %Check:
        if (~isempty(iterError))
            error('ASRL:rawIterCostFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum - 1, filename, iterError);
        end
        if (~isempty(costError))
            error('ASRL:rawIterCostFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum, filename, costError);
        end
        
        %Check that they both have the same number of entries
        if any(any(size(iterData) ~= size(costData)))
            error('ASRL:rawIterCostFile:BadData', 'Unexpectedly found a different number of entries between line %d and line %d in %s.', lineNum - 1, lineNum, filename);
        end
        
        %Check if this run is for longer iters than us:
        if (iterData(end) > resultIter(end))
            resultIter = iterData';
        end
            
        %Store the iter and cost data, the data is in a column and we wish to store it as a row.
        iterCell{end+1, 1} = iterData';
        costCell{end+1, 1} = costData';
    end
    
    fclose(fid);
    
    %Now, prepare the outputs
    resultCost = minCost + zeros(size(costCell,1), size(resultIter,2));
    
    %Then interate over the cells
    %Check that the iterations are correct:
    for idx = 1:size(iterCell,1)
        if any(any(resultIter(1:size(iterCell{idx},2)) ~= iterCell{idx}))
            error('ASRL:rawIterCostFile:BadData', 'Iteration numbers for trial %d don''t match in %s.', idx, filename);
        end
    end
    %Fill in the costs:
    for idx = 1:size(costCell,1)
        resultCost(idx, 1:size(costCell{idx},2)) = costCell{idx};
    end
end