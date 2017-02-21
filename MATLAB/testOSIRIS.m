initSimulation

payload = 2110;
AtlasV411

site = createLaunchSite('Kennedy');

periapsis = 156;
apoapsis = 181;
inclination = 28.5;
[lan, azm, target] = launchTargeting(site, periapsis, apoapsis, inclination, 2.0);

stage1 = struct('type', 0, 'pitch', 8, 'velocity', 45, 'azimuth', 1);

AV4 = flightManager(vehicle, site, target, 0.2, stage1, 2, [0 0 10], [139 4000; 267 2127]);

telemetry(AV4.powered, AV4.coast, 1);
dbgIntegrals(AV4.powered(2:AV4.n), 2);
trajectory(AV4.powered, AV4.coast, target, 2, 1, 3);

clearvars site periapsis apoapsis inclination lan azm target stage1