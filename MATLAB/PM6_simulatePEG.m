%PM6_simulatePEG.m
%Ascent simulation using Powered Explicit Guidance.
%Dependencies:
%   ascentSimulationPEG.m
%      poweredExplicitGuidance.m
%PARAMETERS - USER INPUT
peg_target = 200;
peg_cycle = 2;
s2_dt = 0.05;
%initial conditions structure; super ugly but we'll be reworking that soon
init = struct('t',50,...%s1_results.Plots.t(2780),...
            'alt',6460399,...%s1_results.Plots.altitude(2780),...
            'rad',0,...%s1_results.Plots.radial(2780),...
            'vx',1986.9,...%s1_results.Plots.vx(2780),...
            'type',1,...
            'wind',322.4249,...
            'vy',1068.0);%s1_results.Plots.vy(2780));
%run PEG
s2_results1=ascentSimulation(s2_vehicle, init, struct('type',2,'target',peg_target,'major',peg_cycle), s2_dt);
%plots (this will be largely reworked soon: PEG will exit on cutoff,
%returning only its flight data; free flight for coast periods simulation
%will be added => plot function for colored trajectories will be created)
figure(6); clf;
subplot(1,2,1); plot(s2_results1.Plots.t, s2_results1.Plots.alt); title('Altitude');
subplot(1,2,2); plot(s2_results1.Plots.t, s2_results1.Plots.pitch); title('Pitch');