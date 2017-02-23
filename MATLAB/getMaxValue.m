function [max_i, max_val] = getMaxValue(array)
%[index, value] = GETMAXVALUE(array)
%Finds (the first occurence of) maximum value in a 1D array.
%
%INPUT
%    array      1D array (there is no safety check on that)
%
%OUTPUT
%    index      Index of the maximum element.
%    value      Value of the maximum element.

    max_val = max(array);
    idx = 0;
    for i=1:length(array)
        if array(i)==max_val
            idx = i;
            break;
        end;
    end;
    max_i = idx;