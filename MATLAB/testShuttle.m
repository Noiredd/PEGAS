initSimulation

SpaceShuttle

site = createLaunchSite('Kennedy');

periapsis = 100;
apoapsis = 250;
inclination = 51.65;
[lan, azm, target] = launchTargeting(site, periapsis, apoapsis, inclination, 2.0);

stage1 = struct('type', 0, 'pitch', 8.5, 'velocity', 50, 'azimuth', azm);

STS = flightManager(vehicle, site, target, 0.2, stage1, 2, 0, []);

telemetry(STS.powered, STS.coast, 1);
dbgIntegrals(STS.powered(2:STS.n), 2);
trajectory(STS.powered, STS.coast, target, 2, 1, 3);

clearvars site periapsis apoapsis inclination lan azm target stage1