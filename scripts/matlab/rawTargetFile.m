function [resultTarget, resultTime] = rawTargetFile(filename)
    %Start empty
    resultTarget = [];
    timeCell = {};
    
    %Open the file
    [fid theMessage] = fopen(filename);
	if fid < 0
		error('ASRL:rawTargetFile:FileNotOpened', 'The file ''%s'' could not be opened because: %s',filename,theMessage);
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
        targetString = fgetl(fid);
        timeString = fgetl(fid);
    
        %Break cleanly if it's the end of file
        if ~ischar(targetString) && ~ischar(timeString)
            break;
        elseif ~ischar(targetString) || ~ischar(timeString)
            error('ASRL:rawTargetFile:NumberOfLines', 'Unexpectedly found an odd number of lines in %s.', filename);
        end
        
        %Find the deliminators in the string
        targDelimPos = strfind(targetString, delimiter);
        timeDelimPos = strfind(timeString, delimiter);
        
        %Check their labels
        if (strcmp( targetString(1:targDelimPos(skipColumns)), timeString(1:timeDelimPos(skipColumns)) ) == false)
            error('ASRL:rawTargetFile:BadData', 'The header of lines %d--%d do not match in %s.', lineNum-1, lineNum, filename);
        end
        
        %Crop the strings
        croppedTarg = targetString(targDelimPos(skipColumns):end);
        croppedTime = timeString(timeDelimPos(skipColumns):end);
        
        %Process the strings:
        [targData,~,targError,~] = sscanf(croppedTarg, ',%f');
        [timeData,~,timeError,~] = sscanf(croppedTime, ',%f');
        
        %Check:
        if (~isempty(targError))
            error('ASRL:rawTargetFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum - 1, filename, targError);
        end
        if (~isempty(timeError))
            error('ASRL:rawTargetFile:StringFormat', 'Line %d of file ''%s'' could not be processed because: %s',lineNum, filename, timeError);
        end
        
        %Check that they both have the same number of entries
        if any(any(size(targData) ~= size(timeData)))
            error('ASRL:rawTargetFile:BadData', 'Unexpectedly found a different number of entries between line %d and line %d in %s.', lineNum - 1, lineNum, filename);
        end
        
        %Check that the target data is the same as found so far
        if (isempty(resultTarget))
            resultTarget = targData';
        else
            if any(resultTarget ~= targData')
                error('ASRL:rawTargetFile:BadData', 'Line %d has different target data than previously recorded in in %s.', lineNum - 1, filename);
            end
        end
            
        %Now, store the time data, the data is in a column and we wish to store it as a row.
        timeCell{end+1, 1} = timeData';
    end
    
    fclose(fid);
    
    %Now, prepare the outputs
    resultTime = zeros(size(timeCell,1), size(resultTarget,2));
    
    %Then interate over the cells
    for idx = 1:size(timeCell,1)
        resultTime(idx, 1:size(timeCell{idx},2)) = timeCell{idx};
    end
end