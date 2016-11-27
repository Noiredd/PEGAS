function [v] = unit(vector)
    if norm(vector)==0
        v = vector;
    else
        v = vector/norm(vector);
    end;
end