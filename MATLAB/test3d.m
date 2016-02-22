%test3D
s2_dt = 0.05;
a=flightSim3D(s1_vehicle, s1_init, struct('type', 1, 'program', s1_prog, 'azimuth', 90), s2_dt);
b=flightSim3D(s2_vehicle, resultsToInit(a),...
              struct('type',3, 'length',25), s2_dt);
c=flightSim3D(s2_vehicle, resultsToInit(b),...
              struct('type',2, 'target',200, 'major',2, 'azimuth',90), s2_dt);
telemetry([a, c], b, 1);
trajectory([a, c], b, 1, 1, 2);