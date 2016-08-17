fs_dt = 0.25;
fs_azimuth = 90;    %launch azimuth for old PEG [deg] from North to East
fs_target = 250;    %target orbit altitude [km] ASL
fs_cycle = 2;       %PEG cycle length [s]
fs_coast = 25;      %coast between stages
p_init = s1_init;
p_init.lat = 0;
p_init.lon = 0;
p_init.alt = 0;
%p_stage1 = flightSim3D(s1_vehicle, p_init,...
%    struct('type',1, 'program',s1_prog, 'azimuth',fs_azimuth), fs_dt);
%p_coast1 = flightSim3D(s2_vehicle, resultsToInit(p_stage1),...
%    struct('type',5, 'length',fs_coast), fs_dt);
%p_stage2 = flightSim3D(s2_vehicle, resultsToInit(p_coast1),...
%    struct('type',2, 'target',fs_target, 'major',fs_cycle, 'azimuth',fs_azimuth), fs_dt);
p_target = struct('radius', R+fs_target*1000, 'normal', [0,0,-1],...
                  'angle', 0, 'velocity', sqrt(mu/(R+fs_target*1000)));
if 1
p_stage2 = flightSim3D(s2_vehicle, resultsToInit(p_coast1),...
    struct('type',3, 'target', p_target, 'major', fs_cycle), fs_dt);
end
%p_coast2 = flightSim3D(s2_vehicle, resultsToInit(p_stage2),...
%    struct('type',5, 'length', 6000), 1);
%trajectory([p_stage1, p_stage2], [p_coast1, p_coast2], 1, 1, 2);
%trajectory([p_stage1, p_stage2], [p_coast1], 1, 1, 2);
telemetry(p_stage2, [], 3);
clearvars fs_dt fs_azimuth fs_target fs_cycle fs_coast p_init

%trajectory plotting with thrust vector at time KT
KT = 200;   %pass simulation time [s]
if KT
    trajectory(p_stage2, [], 0, 1, 2);
    figure(2); hold on;
    %find closest index in time plot
    for ki1=1:length(p_stage2.Plots.t)
        if p_stage2.Plots.t(ki1)>KT
            break;
        end;
    end;
    %same for debug plot
    for ki2=1:length(p_stage2.Plots.DEBUG.time)
        if p_stage2.Plots.DEBUG.time(ki2)>KT
            break;
        end;
    end
    %now make plots with that xD
    kv = zeros(2,3);
    kv(1,:) = p_stage2.Plots.r(ki1,:);
    kv(2,:) = kv(1,:) + 200000*p_stage2.Plots.DEBUG.iF(ki2,:);
    plot3(kv(:,1), kv(:,2), kv(:,3), 'k');
    hold off;
    clearvars KT ki1 ki2 kv
end