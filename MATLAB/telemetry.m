%telemetry.m
%Flight data postprocessing and plots. Can separate powered ascent and
%coast flights. Pass target figure ID in 'fid'.
%Able to handle legacy simulation results from flightSim2D, although the
%downrange distance plot will only display results from first powered phase
%and no other.
function [] = telemetry(powered, coast, fid)
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
            dd = zeros(length(x.r),1);
            for j=1:length(x.r)
                dd(j) = dot(zero,x.r(j,:))/(1*x.rmag(j));
                dd(j) = acosd( min(max(dd(j),-1),1) );
                dd(j) = 2*pi*R*dd(j)/360;
            end;
            plot(x.t, dd/1000, c);
        end;
        for i=1:length(coast)
            x = coast(i).Plots;
            dd = zeros(length(x.r),1);
            for j=1:length(x.r)
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