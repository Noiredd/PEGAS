function [y] = approxFromCurve(x, curve)
%y = APPROXFROMCURVE(x, curve)
%Interpolates a point on a curve given by list of control points.
%If the input is between two points of the curve, output will be
%calculated from the linear function fitted to those points. If the input
%is out of range, extreme values will be output.
%
%INPUT
%    x          Argument
%    curve      Array of 2D (XY) points of shape (n,2)
%
%OUTPUT
%   y           Linear approximation for the given argument.

    [n,~] = size(curve);
    
    %If the input is outside the given range, output the extreme value.
    if (x>curve(n,1))
        y = curve(n,2);
        return;
    elseif (x<curve(1,1))
        y = curve(1,2);
        return;
    end;
    
    %Find between which two points defining the curve the argument is.
    for i = 2:n
        if curve(i,1) > x
            break;
        end;
    end;
    %Calculate a linear function coefficients between those points from the
    %given rearrangement:
    %    m*x1+b = y1
    %    m*x2+b = y2
    %    y1-m*x1  =  y2-m*x2
    %    m(x2-x1) = y2-y1
    %    m = (y2-y1)/(x2-x1)
    %    b = y2 - m*x2
    m = ( curve(i,2)-curve(i-1,2) ) / ( curve(i,1)-curve(i-1,1) );
    b = curve(i,2) - m*curve(i,1);
    
    y = m*x+b;