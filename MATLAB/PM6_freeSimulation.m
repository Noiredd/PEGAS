%PM6_freeSimulation.m
%Complete 3D simulation, from launch through in-between-stages coast phase,
%to PEG guided stage and an optional post-MECO coast.
%Dependencies:
%   flightSim3D.m
%   resultsToInit.m
%   telemetry.m
%   trajectory.m
%PARAMETERS - USER INPUT
fs_dt = 0.05;
fs_azimuth = 90;    %CONSTANT LAUNCH AZIMUTH [deg] from North to East
fs_target = 200;    %target orbit altitude [km] ASL
fs_cycle = 2;       %PEG cycle length [s]
fs_coast = 25;      %coast between stages
fs_post = 0;        %coast after MECO
fs_renderE = 1;     %Earth rendering for trajectory display:
                    % 0 = no Earth, 1 = only upper hemi, 2 = whole Earth
fs_renderV = 1;     %vectors rendering for trajectory display:
                    % 0 = no vectors, 1 = only last powered stage, 2 = all
%SIMULATIONS
%first stage
fs_stage1 = flightSim3D(s1_vehicle,...
    s1_init,...
    struct('type',1, 'program',s1_prog, 'azimuth',fs_azimuth),...
    fs_dt);
%coast
fs_coast1 = flightSim3D(s2_vehicle,...
    resultsToInit(fs_stage1),...
    struct('type',3, 'length',fs_coast),...
    fs_dt);
%second stage
fs_stage2 = flightSim3D(s2_vehicle,...
    resultsToInit(fs_coast1),...
    struct('type',2, 'target',fs_target, 'major',fs_cycle, 'azimuth',fs_azimuth),...
    fs_dt);
%optional final coast
if fs_post>0
    fs_coast2 = flightSim3D(s2_vehicle,...
        resultsToInit(fs_stage2),...
        struct('type',3, 'length',fs_post),...
        fs_dt);
end;
%PLOTS
if fs_post>0
    c = [fs_coast1, fs_coast2];
else
    c = fs_coast1;
end;
telemetry([fs_stage1, fs_stage2], c, 5);
trajectory([fs_stage1, fs_stage2], c, fs_renderE, fs_renderV, 6);
%cleanup
clearvars fs_dt fs_azimuth fs_target fs_cycle fs_coast fs_post
clearvars fs_renderE fs_renderV fs_coast1 fs_coast2 c