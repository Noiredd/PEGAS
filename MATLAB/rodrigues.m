function [rotated] = rodrigues(vector, axis, angle)
    %https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
    axis = unit(axis);
    rotated = vector*cosd(angle);
    rotated = rotated + cross(axis, vector)*sind(angle);
    rotated = rotated + axis*dot(axis,vector)*(1-cosd(angle));