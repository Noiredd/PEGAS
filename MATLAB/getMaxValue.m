%getMaxValue.m
%Finds maximum value of an array and its index (first occurence) within it.
function [max_i, max_val] = getMaxValue(array)
    max_val = max(array);
    idx = 0;
    [~,n]=size(array);
    for i=1:n
        if array(i)==max_val
            idx = i;
            break;
        end;
    end;
    max_i = idx;