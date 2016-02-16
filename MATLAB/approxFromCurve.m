%approxFromCurve.m
%Given a curve (as a list of 2D points) and an argument (number)
%approximates a linear function from two points between which the argument
%is and calculates its output for the number as argument.
function [y] = approxFromCurve(x, curve)
    %some safety checks
    [n,~] = size(curve);
    %find between which two points defining the curve we are
    for i = 2:n
        if curve(i,1) > x
            break;
        end;
    end;
    %find linear function between those points
        %m*x1+b=y1
        %m*x2+b=y2
        %y1-m*x1 = y2-m*x2
        %m(x2-x1)=y2-y1
    m = ( curve(i,2)-curve(i-1,2) ) / ( curve(i,1)-curve(i-1,1) );
    b = curve(i,2) - m*curve(i,1);
    %compute
    y = m*x+b;