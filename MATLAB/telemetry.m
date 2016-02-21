%telemetry.m
%Flight data postprocessing and plots. Can separate powered ascent and
%coast flights. Pass target figure ID in 'fid'.
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
        plot(x.t, x.angle_ps, 'g');
        plot(x.t, x.angle_po, 'g');
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        plot(x.t, x.pitch, 'b');
        plot(x.t, x.angle_ps, 'g');
        plot(x.t, x.angle_po, 'g');
    end;
    title('Pitch and flight angles');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    hold off;
    %altidute
    subplot(2,3,2); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        plot(x.t, (x.rmag-R)/1000, c);
    end;
    for i=1:length(coast)
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
        plot(x.t, x.a/g0, c);
    end;
    for i=1:length(coast)
        x = coast(i).Plots;
        plot(x.t, x.a/g0, 'b');
    end;
    title('Acceleration');
    xlabel('Time [s]');
    ylabel('Acceleration [g]');
    hold off;
    %downrange distance
    subplot(2,3,5); hold on;
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
    title('Downrange');
    xlabel('Time [s]');
    ylabel('Distance [km]');
    hold off;
    %velocity T
    subplot(2,3,6); hold on;
    for i=1:length(powered)
        x = powered(i).Plots;
        plot(x.t, x.vt, c);
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