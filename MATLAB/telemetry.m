function [] = telemetry(powered, coast, fid)
%TELEMETRY(powered, coast, figure)
%Plots basic flight data in 6 subplots in a given figure:
%   pitch and yaw commands with flight path angle (orbital and surface)
%   altitude
%   vertical velocity
%   acceleration
%   downrange distance
%   horizontal velocity
%Has some basic support for legacy results from flightSim2D.
%
%REQUIRES
%    g0         Global variable, standard gravity acceleration (m/s).
%    R          Global variable, radius of the body (m).
%
%INPUT
%    powered    Array of struct of type results, as output by flightSim3D,
%               corresponding to powered phases.
%    coast      Array of struct of type results, as output by flightSim3D,
%               corresponding to coast phases.
%               Best to just extract the respective fields from a combined
%               results struct as output by flightManager.
%    figure     ID of a figure in which to plot.
%
%OUTPUT
%    (none)
%
%See also FLIGHTSIM3D, FLIGHTMANAGER.

    global R;
    global g0;
    %if no coast periods are to be shown, don't distinguish
    if isempty(coast)
        c = 'b';
    else
        c = 'r';
    end;
    figure(fid), clf;
    %pitch&flight angle
    subplot(2,3,1); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        plot(x.t, x.pitch, 'b');
        plot(x.t, x.yaw, 'y');
        plot(x.t, x.angle_ps, 'g');
        plot(x.t, x.angle_po, 'g');
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        %plot(x.t, x.pitch, 'b');
        plot(x.t, x.angle_ps, 'g');
        plot(x.t, x.angle_po, 'g');
    end;
    title('Pitch and flight angles');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    hold off;
    %altitude - HANDLE LEGACY RESULTS (flightSim2D.m)
    subplot(2,3,2); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        %legacy plot has no 'rmag' field but an 'alt' one
        if isfield(x, 'alt')
            plot(x.t, x.alt/1000, c);
        else
            plot(x.t, (x.rmag-R)/1000, c);
        end;
    end;
    for i=1:length(coast)
        %coast results expected to only come from 3D simulation
        x = coast(i).Plots;
        plot(x.t, (x.rmag-R)/1000, 'b');
    end;
    title('Altitude');
    xlabel('Time [s]');
    ylabel('Altitude ASL [km]');
    hold off;
    %velocity Y
    subplot(2,3,3); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        plot(x.t, x.vy, c);
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        plot(x.t, x.vy, 'b');
    end;
    title('Vertical velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    hold off;
    %acceleration
    subplot(2,3,4); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        plot(x.t, x.a/g0, 'b');
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        plot(x.t, x.a/g0, 'b');
    end;
    title('Acceleration');
    xlabel('Time [s]');
    ylabel('Acceleration [g]');
    hold off;
    %downrange distance - HANDLE LEGACY RESULTS (flightSim2D.m)
    %For legacy results will only display results from first powered phase.
    subplot(2,3,5); hold on;
    if isfield(powered(1).Plots, 'rad')
        %this returns true for legacy structure
        plot(powered(1).Plots.t, (powered(1).Plots.rad-powered(1).Plots.rad(1))*(2*pi*R/360000), c);
    else
        %normal handling for pure 3D results
        zero = powered(1).Plots.r(1,:);
        zero = zero/norm(zero);
        for i=1:length(powered)
            x = powered(i).Plots;
            dd = zeros(length(x.r(:,1)),1);
            for j=1:length(x.r(:,1))
                %Calculates angular distance from launch site, then length
                %of the arc on the surface of the Earth. Therefore this is
                %NOT the total length of the burn arc!
                dd(j) = dot(zero,x.r(j,:))/(1*x.rmag(j));
                dd(j) = acosd( min(max(dd(j),-1),1) );
                dd(j) = 2*pi*R*dd(j)/360;
            end;
            plot(x.t, dd/1000, c);
        end;
        for i=1:length(coast)
            x = coast(i).Plots;
            dd = zeros(length(x.r(:,1)),1);
            for j=1:length(x.r(:,1))
                dd(j) = dot(zero,x.r(j,:))/(1*x.rmag(j));
                dd(j) = acosd( min(max(dd(j),-1),1) );
                dd(j) = 2*pi*R*dd(j)/360;
            end;
            plot(x.t, dd/1000, 'b');
        end;
    end;
    title('Downrange');
    xlabel('Time [s]');
    ylabel('Distance [km]');
    hold off;
    %velocity T - HANDLE LEGACY RESULTS (flightSim2D.m)
    subplot(2,3,6); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        if isfield(x, 'vx')
            %legacy results have no 'vt' field, but a 'vx' one
            plot(x.t, x.vx, c);
        else
            plot(x.t, x.vt, c);
        end;
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        plot(x.t, x.vt, 'b');
    end;
    title('Horizontal velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    hold off;
    %remove excessive margins
    tightfig;
end