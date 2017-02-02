initSimulation
AtlasV531

site = createLaunchSite('Plesetsk');

alt = 200;
inc = 90;
[lan, azm, target] = launchTargeting(site, alt, alt, inc, 2.0);

stage1 = struct('type', 0, 'p', 10, 'v', 50, 'a', azm);

AV5 = flightManager(vehicle, site, target, 0.2, stage1, 2, 5, []);

telemetry(AV5.powered, AV5.coast, 1);
dbgIntegrals(AV5.powered(2:AV5.n), 2);
trajectory(AV5.powered, AV5.coast, target, 2, 1, 3);

fprintf('injection at t+%.1f into %.1fx%.1fkm orbit\n', max(AV5.powered(AV5.n).Plots.t), AV5.powered(AV5.n).Apoapsis, AV5.powered(AV5.n).Periapsis);
fprintf('angle %d\tecc %d\talt %d\n', planeError(AV5, target), AV5.powered(AV5.n).Orbit.ECC, abs(alt-AV5.powered(AV5.n).Altitude));

clearvars site alt inc lan azm target stage1