function [] = dbgIntegrals(stages, fid)
%DBGINTEGRALS(results, figure)
%Plots UPFG thrust integrals from up to 4 given stages.
%
%INPUT
%    results    Array of struct of type results (as output by flightSim3D).
%               Each must contain a field 'Plots' being a struct with a
%               field 'DEBUG'.
%    figure     ID of a figure in which to plot.
%
%OUTPUT
%    (none)

    figure(fid), clf;
    colors = ['b', 'r', 'g', 'y'];
    
    %Plot of the last stage ends abruptly (time is left unset at zero).
    %This has to be fixed, else plots get weird.
    maxes = [0];
    for i=1:length(stages)
        A = stages(i).Plots.DEBUG.time;
        maxes(i) = find(A==max(A(:)));
    end
    
    subplot(2,4,1); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.L(2:maxes(i)), colors(i));
    end
    title('L integral');
    hold off;
    
    subplot(2,4,2); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.J(2:maxes(i)), colors(i));
    end
    title('J integral');
    hold off;
    
    subplot(2,4,3); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.S(2:maxes(i)), colors(i));
    end
    title('S integral');
    hold off;
    
    subplot(2,4,5); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.Q(2:maxes(i)), colors(i));
    end
    title('Q integral');
    hold off;
    
    subplot(2,4,6); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.P(2:maxes(i)), colors(i));
    end
    title('P integral');
    hold off;
    
    subplot(2,4,7); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.H(2:maxes(i)), colors(i));
    end
    title('H integral');
    
    subplot(2,4,4); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.vgo2(2:maxes(i),4), colors(i));
    end
    title('Vgo (2)');
    
    subplot(2,4,8); hold on;
    for i=1:length(stages)
        plot(stages(i).Plots.DEBUG.time(2:maxes(i)), stages(i).Plots.DEBUG.tgo(2:maxes(i)), colors(i));
    end
    title('Tgo');
    hold off;