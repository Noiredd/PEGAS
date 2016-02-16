%createLineApprox.m
%Approximates a given scattered-data curve given by a 2xN 'raw' vector
%with n+2 line segments. Designed to work with a following curve scheme:
% - flat segment at 0
% - linear rise to a given value
% - flat segment at that value
% - free turn, approximated with n-1 segments
%Dependencies:
%   linearFit.m
function [app] = createLineApprox(raw, n)
    %initialize variables (sets first point to 0,0)
    app = zeros(2, n+3); %n+3 points yield n+2 segments
    c = 2; %outer iterator (between segments)
    %easy part - get the first three segments
    for i = c:length(raw)
        %first, flat segment
        if raw(2,i) > 0
            app(1,2) = raw(1,i);    %second point time component
            app(2,2) = 0;           %pitch is still zero
            c = i;
            break;
        end;
    end;
    for i = c:length(raw)
        %second, rise segment
        if raw(2,i) == raw(2,i+1)
            app(1,3) = raw(1,i);    %third point time
            app(2,3) = raw(2,i);    %pitch has risen
            c = i+1;
            break;
        end;
    end;
    for i = c:length(raw)
        %finally, last flat segment
        if raw(2,i) > raw(2,c)
            app(1,4) = raw(1,i);    %fourth point time
            app(2,4) = raw(2,c);    %pitch has remained constant
            c = i;
            break;
        end;
    end;
    %tricky part - linear fit to rest of the data
    fit = linearFit(raw(:,c:length(raw)), n);
    for i = 2:n
        app(1,i+3) = fit(1,i);
        app(2,i+3) = fit(2,i);
    end;
end