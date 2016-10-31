SpaceShuttle

site = struct('type', 0,'lat', 28.52406, 'lon', -80.65085, 'alt', 0);   %KSC
[lan, azm, target] = launchTargeting(site, 200, 51.65, 2.0);

stage1 = struct('type', 0, 'p', 10, 'v', 50, 'a', azm);

STS = flightManager(vehicle, site, target, 0.2, stage1, 2, 5);

telemetry(STS.powered, STS.coast, 1);
dbgIntegrals(STS.powered(2:STS.n), 2);
trajectory(STS.powered, STS.coast, target, 2, 1, 3);

planeError(STS, target);

clearvars site lan azm target stage1