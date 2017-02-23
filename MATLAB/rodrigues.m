function [rotated] = rodrigues(vector, axis, angle)
%[rotated] = RODRIGUES(vector, axis, angle)
%Rotates a given vector about a given axis by a given angle using Rodrigues
%rotation formula:
%https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
%
%INPUT
%    vector     XYZ vector to be rotated.
%    axis       XYZ vector representing the axis of rotation (will be
%               normalized so its magnitude does not matter).
%    angle      Angle of rotation (degrees).
%
%OUTPUT
%    rotated    Rotated XYZ vector.

    axis = unit(axis);
    rotated = vector*cosd(angle);
    rotated = rotated + cross(axis, vector)*sind(angle);
    rotated = rotated + axis*dot(axis,vector)*(1-cosd(angle));