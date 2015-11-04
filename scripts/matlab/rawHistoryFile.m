function [resultTime, resultCost] = rawHistoryFile(filename,maxTime)
    %Start with empty cells
    timeCell = {};
    costCell = {};
    %And no output columns
    outputColSize = 0;
    
    %Open the file
    [fid theMessage] = fopen(filename);
	if fid < 0
		error('ASRL:rawHistoryFile:FileNotOpened', 'The file ''%s'' could not be opened because: %s',filename,theMessage);
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
        timeString = fgetl(fid);
        costString = fgetl(fid);
    
        %Break cleanly if it's the end of file
        if ~ischar(timeString) && ~ischar(costString)
            break;
        elseif ~ischar(timeString) || ~ischar(costString)
            error('ASRL:rawHistoryFile:NumberOfLines', 'Unexpectedly found an odd number of lines in %s.', filename);
        end
        
        %Find the deliminators in the string
        timeDelimPos = strfind(timeString, delimiter);
        costDelimPos = strfind(costString, delimiter);
        
        %Check their labels
        if (strcmp( timeString(1:timeDelimPos(skipColumns)), costString(1:costDelimPos(skipColumns)) ) == false)
            error('ASRL:rawHistoryFile:BadData', 'The header of lines %d--%d do not match in %s.', lineNum-1, lineNum, filename);
        end
        
        %Crop the strings
        croppedTime = timeString(timeDelimPos(skipColumns):end);
        croppedCost = costString(costDelimPos(skipColumns):end);
        
        %Process the strings:
        [timeData,~,timeError,~] = sscanf(croppedTime, ',%f');
        [costData,~,costError,~] = sscanf(croppedCost, ',%f');
        
        %Check:
        if (~isempty(timeError))
            error('ASRL:rawHistoryFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum - 1, filename, timeError);
        end
        if (~isempty(costError))
            error('ASRL:rawHistoryFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum, filename, costError);
        end
        
        %Check that they both have the same number of entries
        if any(any(size(timeData) ~= size(costData)))
            error('ASRL:rawHistoryFile:BadData', 'Unexpectedly found a different number of entries between line %d and line %d in %s.', lineNum - 1, lineNum, filename);
        end
        
        %Now, only store time values less than the max-time parameter, the data is in a column and we wish to store it as a row.
        timeCell{end+1, 1} = timeData(timeData <= maxTime)';
        costCell{end+1, 1} = costData(timeData <= maxTime)';
        
        %Update the size variable
        outputColSize = max(outputColSize, size(timeCell{end, 1},2));
    end
    
    fclose(fid);
    
    %Now, prepare the outputs
    resultTime = zeros(size(timeCell,1), outputColSize);
    resultCost = zeros(size(costCell,1), outputColSize);
    
    %Then interate over the cells
    for idx = 1:size(timeCell,1)
        resultTime(idx, 1:size(timeCell{idx},2)) = timeCell{idx};
        resultCost(idx, 1:size(costCell{idx},2)) = costCell{idx};
    end
end