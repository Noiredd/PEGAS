%PM6_simulatePEG.m
%Ascent simulation using Powered Explicit Guidance.
%Dependencies:
%   ascentSimulationPEG.m
%      poweredExplicitGuidance.m
%   continueFlight.m
%PARAMETERS - USER INPUT
peg_target = 200;
peg_cycle = 2;
s2_dt = 0.05;
%initial conditions structure: starting from arbitrary point is possible,
%but so is easy continuation from results of previous stage simulation
init = struct('t',50,...
              'alt',6460399,...
              'rad',0,...
              'vx',1986.9,...
              'type',1,...
              'wind',322.4249,...
              'vy',1068.0);
init = continueFlight(s1_results, 322.4249);
%run PEG
s2_results1=ascentSimulation(s2_vehicle, init, struct('type',2,'target',peg_target,'major',peg_cycle), s2_dt);
%plots will be largely reworked soon:
%free flight for coast periods simulation will be added
%plot function for colored trajectories will be created
figure(6); clf;
subplot(1,2,1); plot(s2_results1.Plots.t, s2_results1.Plots.alt); title('Altitude');
subplot(1,2,2); plot(s2_results1.Plots.t, s2_results1.Plots.pitch); title('Pitch');