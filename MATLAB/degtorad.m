function radians = degtorad(degrees)
%radians = DEGTORAD(degrees)
%Converts angle in degrees into radians. Useful to users who do not own the
%Mapping Toolbox.
%
%INPUT
%    degrees    Angle in degrees.
%
%OUTPUT
%    radians    The same angle in radians.

radians = degrees * pi/180;