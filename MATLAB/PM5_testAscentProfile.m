%PM5_testAscentProfile.m
%Ascent simulation using precalculated pitch program.
%Dependencies:
%   ascentSimulationPEG.m
%   flightPlots.m
%PARAMETERS - USER INPUT
s1_dt = 0.05;           %sim precision [s]
%run simulation
s1_prog = pp_prog';
s1_results = ascentSimulation(s1_vehicle, s1_init, struct('type', 1, 'program', s1_prog), s1_dt);
%and show plots
flightPlots(s1_results, [], 4);