function [v] = unit(vector)
%u = UNIT(v)
%Returns a normalized vector. If zero vector is passed, returns this vector.
%
%INPUT
%    v          Any valid XYZ vector.
%
%OUTPUT
%    u          Vector of the same direction as v but of magnitude 1.

    if norm(vector)==0
        v = vector;
    else
        v = vector/norm(vector);
    end;