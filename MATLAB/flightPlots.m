%flightPlots.m
%Nice, colored plots for the whole ascent.
%Pass simulation results for powered and coast phases in respective structs.
%Pass figure ID in 'fid'.
function [] = flightPlots(powered, coast, fid)
    global R; %recall Earth radius for downrange distance calculation
    global g0; %recall standard gravity acceleration for convertion into Gs
    %if there are no coast periods to show, no point in distinguishing
    %powered flight from them
    if isempty(coast)
        clr = 'b';
    else
        clr = 'r';
    end;
    figure(fid); clf;
    %pitch&angle plots
    subplot(2,3,1);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, powered(i).Plots.pitch, 'b');
        plot(powered(i).Plots.t, powered(i).Plots.angle_srf, 'g');
        plot(powered(i).Plots.t, powered(i).Plots.angle_obt, 'g');
    end;
    for i=1:length(coast)
        plot(coast(i).Plots.t, coast(i).Plots.pitch, 'b');
        plot(coast(i).Plots.t, coast(i).Plots.angle_srf, 'g');
        plot(coast(i).Plots.t, coast(i).Plots.angle_obt, 'g');
    end;
    title('Pitch and flight angles');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    hold off;
    %altitude plots
    subplot(2,3,2);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, powered(i).Plots.alt/1000, clr);
    end;
    for i=1:length(coast)
        plot(coast(i).Plots.t, coast(i).Plots.alt/1000, 'b');
    end;
    title('Altitude');
    xlabel('Time [s]');
    ylabel('Altitude ASL [km]');
    hold off;
    %velocity plots Y
    subplot(2,3,3);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, powered(i).Plots.vy, clr);
    end;
    for i=1:length(coast)
        plot(coast(i).Plots.t, coast(i).Plots.vy, 'b');
    end;
    title('Vertical velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    hold off;
    %acceleration plots
    subplot(2,3,4);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, powered(i).Plots.a/g0, 'b');
    end;
    for i=1:length(coast)
        %show it anyway, just to illustrate length of the coast phase
        plot(coast(i).Plots.t, coast(i).Plots.a/g0, 'b');
    end;
    title('Acceleration');
    xlabel('Time [s]');
    ylabel('Acceleration [g]');
    hold off;
    %downrange distance plots
    subplot(2,3,5);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, (powered(i).Plots.rad-powered(1).Plots.rad(1))*(2*pi*R/360000), clr);
    end;
    for i=1:length(coast)
        plot(coast(i).Plots.t, (coast(i).Plots.rad-powered(1).Plots.rad(1))*(2*pi*R/360000), 'b');
    end;
    title('Downrange');
    xlabel('Time [s]');
    ylabel('Distance [km]');
    hold off;
    %velocity plots X
    subplot(2,3,6);
    hold on;
    for i=1:length(powered)
        plot(powered(i).Plots.t, powered(i).Plots.vx, clr);
    end;
    for i=1:length(coast)
        plot(coast(i).Plots.t, coast(i).Plots.vx, 'b');
    end;
    title('Horizontal velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    hold off;
end