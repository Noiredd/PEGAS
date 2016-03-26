%PM5_testAscentProfile.m
%Ascent simulation using precalculated pitch program.
%Dependencies:
%   flightSim2D.m       legacy 2D simulation, because faster than 3D
%                       consider complete migration to 3D for consistency
%   telemetry.m
%PARAMETERS - USER INPUT
s1_dt = 0.05;           %sim precision [s]
%run simulation
s1_prog = pp_prog';
s1_results = flightSim2D(s1_vehicle, s1_init, struct('type', 1, 'program', s1_prog), s1_dt);
%and show plots
telemetry(s1_results, [], 4);