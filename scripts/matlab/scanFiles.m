function scanFiles(filename, expectedPattern, stopOnError, runQuietly)
    if nargin < 3
        stopOnError = true;
    end
    
    if nargin < 4
        runQuietly = false;
    end
    
    %If we're not outputing info, we must stop on errors:
    if runQuietly == true
        stopOnError = true;
    end

    %Open the file
    [fid theMessage] = fopen(filename);
	if fid < 0
		error('ASRL:scanFileLabels:FileNotOpened', 'The file ''%s'' could not be opened because: %s',filename,theMessage);
	end
    
    delimiter = sprintf(',');
    skipColumns = 1;
    %If you wanted to skip rows, you'd need to just jump over processing an appropriate number of times
    
    %Iterate over the lines
    lineNum = 0;
    tokenNum = 1;
    while 1
        %Increment our line number
        lineNum = lineNum + 2;
        
        %Get the lines
        string1 = fgetl(fid);
        string2 = fgetl(fid);
    
        %Break cleanly if it's the end of file
        if ~ischar(string1) && ~ischar(string2)
            break;
        elseif ~ischar(string1) || ~ischar(string2)
            error('ASRL:scanFileLabels:NumberOfLines', 'Unexpectedly found an odd number of lines in %s.', filename);
        end
        
        %Find the deliminators in the string
        delimPos1 = strfind(string1, delimiter);
        delimPos2 = strfind(string2, delimiter);
        
        if isempty(delimPos1)
            error('ASRL:scanFileLabels:BadData', 'Line %d is not comma-deliminated in %s.', lineNum-1, filename);
        end
        
        if isempty(delimPos2)
            error('ASRL:scanFileLabels:BadData', 'Line %d is not comma-deliminated in %s.', lineNum, filename);
        end
        
        %Get the labels:
        label1 = string1(1:delimPos1(skipColumns)-1);
        label2 = string2(1:delimPos2(skipColumns)-1);
        
        %Check that they're the same:
        if (strcmp( label1, label2 ) == false)
            error('ASRL:scanFileLabels:BadData', 'The header of lines %d--%d (%s and %s) do not match in %s.', lineNum-1, lineNum, label1, label2, filename);
        end
        
        %Check them to the expected sequence:
        if (strcmp( label1, expectedPattern{tokenNum} ) == false)
            errorString = sprintf('The header of lines %d--%d (%s) do not match the expected entry for this line (%s) in %s.', lineNum-1, lineNum, label1, expectedPattern{tokenNum}, filename);
            
            if (stopOnError)
                error('ASRL:scanFileLabels:BadData', errorString);
            else
                warning('ASRL:scanFileLabels:BadData', errorString);
            end
        else
            %Roll the token:
            tokenNum = tokenNum + 1;
            if (tokenNum > size(expectedPattern,1))
                tokenNum = 1;
            end
        end
        
        %Output the labels:
        if (~runQuietly)
            fprintf('Lines %d--%d: %s: %3.3f %3.3f \n', lineNum-1, lineNum, label1, str2double(string1(delimPos1(end):end)), str2double(string2(delimPos2(end):end)));
        end
    end
    
    fclose(fid);
end