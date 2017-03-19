initSimulation

AtlasV531

site = createLaunchSite('Plesetsk');

alt = 200;
inc = 90;
[lan, azm, target] = launchTargeting(site, alt, alt, inc, 2.0);

stage1 = struct('type', 0, 'pitch', 10, 'velocity', 50, 'azimuth', azm);

AV5 = flightManager(vehicle, site, target, 0.2, stage1, 2, 5, []);

telemetry(AV5.powered, AV5.coast, 1);
dbgIntegrals(AV5.powered(2:AV5.n), 2);
trajectory(AV5.powered, AV5.coast, target, 2, 1, 3);

clearvars site alt inc lan azm target stage1