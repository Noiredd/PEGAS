%PM6_simulatePEG.m
%Ascent simulation using Powered Explicit Guidance.
%Dependencies:
%   ascentSimulationPEG.m
%   continueFlight.m
%   flightPlots.m
%PARAMETERS - USER INPUT
peg_target = 200;
peg_cycle = 2;
peg_coast = 25; %coast period before second stage ignition
peg_post = 60;  %show orbit after burnout too
s2_dt = 0.04;
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
preCoast = ascentSimulation(s2_vehicle, init, struct('type',3,'length',peg_coast), s2_dt);
init = continueFlight(preCoast, 322.4249);
%run PEG
s2_results = ascentSimulation(s2_vehicle, init, struct('type',2,'target',peg_target,'major',peg_cycle), s2_dt);
%post coast
init = continueFlight(s2_results, 322.4249);
postCoast = ascentSimulation(s2_vehicle, init, struct('type',3,'length',peg_post), s2_dt);
%plots
flightPlots([s1_results s2_results], [preCoast postCoast], 5);