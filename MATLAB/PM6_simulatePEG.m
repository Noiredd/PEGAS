%PM6_simulatePEG.m
%Ascent simulation using Powered Explicit Guidance.
%Dependencies:
%   poweredExplicitGuidance.m
%PARAMETERS - USER INPUT
c=struct('target',200,'ct',2);  %control data structure (target orbit, PEG major loop cycle length)
%vehicle data package (TODO: move to createVehicle)
v=struct('m0',s2_m0,'isp',s2_isp,'dm',s2_dm,'mt',s2_maxT);
%initial conditions structure; super ugly but we'll be reworking that soon
init=struct('t',0,...%s1_results.Plots.t(2780),...
            'alt',6460399,...%s1_results.Plots.altitude(2780),...
            'rad',0,...%s1_results.Plots.radial(2780),...
            'vx',1986.9,...%s1_results.Plots.vx(2780),...
            'vy',1068.0);%s1_results.Plots.vy(2780));
%run PEG
s2_results=ascentSimulationPEG(v,init,c,0.05);
%plots (this will be largely reworked soon: PEG will exit on cutoff,
%returning only its flight data; free flight for coast periods simulation
%will be added => plot function for colored trajectories will be created)
figure(6); clf;
subplot(1,2,1); plot(s2_results.Plots.t, s2_results.Plots.altitude); title('Altitude');
subplot(1,2,2); plot(s2_results.Plots.t, s2_results.Plots.pitch); title('Pitch');