AtlasV531

%site = struct('type', 0, 'lat', 5.15972, 'lon', -52.65028, 'alt', 0);    %Kourou
%site = struct('type', 0, 'lat', 28.52406, 'lon', -80.65085, 'alt', 0);   %KSC
%site = struct('type', 0, 'lat', 34.75083, 'lon', -120.49778, 'alt', 0);  %Vandenberg
site = struct('type', 0, 'lat', 62.960, 'lon', 40.683, 'alt', 0);        %Plesetsk
[lan, azm, target] = launchTargeting(site, 250, 90, 2.0);

stage1 = struct('type', 0, 'p', 10, 'v', 5, 'a', azm);

AV5 = flightManager(vehicle, site, target, 0.2, stage1, 2, 5);

telemetry(AV5.powered, AV5.coast, 1);
dbgIntegrals(AV5.powered(2:AV5.n), 2);
trajectory(AV5.powered, AV5.coast, target, 2, 1, 3);

fprintf('angle %d\tecc %d\talt %d\n', planeError(AV5, target), AV5.powered(length(AV5.powered)).Orbit.ECC, abs(250-AV5.powered(length(AV5.powered)).Altitude));

clearvars site lan azm target stage1