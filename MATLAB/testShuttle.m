SpaceShuttle

%site = struct('type', 0, 'lat', 5.15972, 'lon', -52.65028, 'alt', 0);    %Kourou
site = struct('type', 0, 'lat', 28.52406, 'lon', -80.65085, 'alt', 0);   %KSC
%site = struct('type', 0, 'lat', 34.75083, 'lon', -120.49778, 'alt', 0);  %Vandenberg
%site = struct('type', 0, 'lat', 62.960, 'lon', 40.683, 'alt', 0);        %Plesetsk
[lan, azm, target] = launchTargeting(site, 200, 51.65, 2.0);

stage1 = struct('type', 0, 'p', 10, 'v', 50, 'a', azm);

STS = flightManager(vehicle, site, target, 0.2, stage1, 2, 5);

telemetry(STS.powered, STS.coast, 1);
dbgIntegrals(STS.powered(2:STS.n), 2);
trajectory(STS.powered, STS.coast, target, 2, 1, 3);

fprintf('angle %d\tecc %d\talt %d\n', planeError(STS, target), STS.powered(length(STS.powered)).Orbit.ECC, abs(250-STS.powered(length(STS.powered)).Altitude));

clearvars site lan azm target stage1