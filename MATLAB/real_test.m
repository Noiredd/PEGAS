%SIMULATION SETUP
fs_dt = 0.25;
fs_cycle = 2;       %PEG cycle length [s]
fs_coast = 25;      %coast between stages

%INITIAL CONDITIONS
%p_init = struct('lat', 5.22167, 'lon', -52.75389, 'alt', 0);   %Kourou
p_init = struct('type', 0,'lat', 28.52406, 'lon', -80.65085, 'alt', 0);   %KSC

%TARGET ORBIT SETUP
%altitude [km ASL]
fs_target = 200;
%inclination (ISS)
inc = 51.65-20;
%launch azimuth calculation
Binertial = asind( cosd(inc)/cosd(p_init.lat) );    %launch azimuth with no regard for Earth rotation
vorbit = sqrt(mu/(R+fs_target*1000));               %orbital velocity magnitude
vEarthrot = 465.101*cosd(p_init.lat);               %v gained from Earth rotation
vrotx = vorbit*sind(Binertial)-vEarthrot;
vroty = vorbit*cosd(Binertial);
fs_azimuth = atan2d(vroty, vrotx);                  %corrected launch azimuth
%LAN: normally we'd launch "a bit" before passing under the target orbit.
%Unfortunately we don't simulate actual Earth rotation (with respect to any
%fixed direction), so we'll just calculate what would it be if there was no
%rotation at all, and then fix at a few (say 3) degrees greater.
lan = p_init.lon - asind(min(1,tand(90-inc)*tand(p_init.lat))) + 2.5;
if lan<360
    lan=lan+360;
end
%To calculate orbital plane normal vector, we rotate a unit vector pointing
%perfect South (negative Z axis - btw please don't ask me why, this just
%works while positive didn't). First about the X axis (corresponding to
%zero longitude meridian) by inclination angle, and then about the Z axis.
Rx=[1,0,0;0,cosd(inc),-sind(inc);0,sind(inc),cosd(inc)];    %about x for inclination (preserve zero node)
Ry=[cosd(inc),0,sind(inc);0,1,0;-sind(inc),0,cosd(inc)];    %in case we needed it for something
Rz=[cosd(lan),-sind(lan),0;sind(lan),cosd(lan),0;0,0,1];    %about z for node
target_iy = (Rz*Rx*[0,0,-1]')';
p_target = struct('radius', R+fs_target*1000, 'normal', target_iy,...
                  'angle', 0, 'velocity', sqrt(mu/(R+fs_target*1000)));

%SIMULATION
if 0    %this is for the old demo rocket as in the youtube video
p_stage1 = flightSim3D(oldicbm, 1, p_init,...
    struct('type',1, 'program',s1_prog, 'azimuth',fs_azimuth), fs_dt);
p_coast1 = flightSim3D(oldicbm, 2, resultsToInit(p_stage1),...
    struct('type',5, 'length',fs_coast), fs_dt);
p_stage2 = flightSim3D(oldicbm, 2, resultsToInit(p_coast1),...
    struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
%p_coast2 = flightSim3D(s2_vehicle, resultsToInit(p_stage2),...
%    struct('type',5, 'length',3000), 1);
telemetry([p_stage1, p_stage2], [p_coast1], 1);
trajectory([p_stage1, p_stage2], [p_coast1], [], 2, 1, 2);
inc_r = p_stage2.Orbit.INC;
lan_r = p_stage2.Orbit.LAN;
end

v3d = 0;    %set 1 to quickly enable all 3D trajectory plots
s = 0;
if ~s       %this is for demo Shuttle
    evSTS
    sts1_1 = flightSim3D(vehicle, 1, p_init,...
        struct('type',0, 'p',10, 'v',50, 'a',90-fs_azimuth), fs_dt);
    sts1_c = flightSim3D(vehicle, 1, resultsToInit(sts1_1),...
        struct('type',5, 'length',5), fs_dt);
    sts1_2 = flightSim3D(vehicle, 2, resultsToInit(sts1_c),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    telemetry([sts1_1, sts1_2], sts1_c, 1);
    if v3d
        trajectory([sts1_1, sts1_2], sts1_c, p_target, 2, 1, 3);
    end;
    dbgIntegrals(sts1_2, 2);
    inc_r = sts1_2.Orbit.INC;
    lan_r = sts1_2.Orbit.LAN;
elseif s<100  %this is for virtual 3-stage Shuttle
    evSTS2
    sts2_1 = flightSim3D(vehicle, 1, p_init,...
        struct('type',0, 'p',10, 'v',50, 'a',90-fs_azimuth), fs_dt);
    sts2_c1 = flightSim3D(vehicle, 1, resultsToInit(sts2_1),...
        struct('type',5, 'length',5), fs_dt);
    sts2_2 = flightSim3D(vehicle, 2, resultsToInit(sts2_c1),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    sts2_c2 = flightSim3D(vehicle, 2, resultsToInit(sts2_2),...
        struct('type',5, 'length',5), fs_dt);
    sts2_3 = flightSim3D(vehicle, 3, resultsToInit(sts2_c2),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    telemetry([sts2_1, sts2_2, sts2_3], [sts2_c1, sts2_c2], 4);
    if v3d
        trajectory([sts2_1, sts2_2, sts2_3], [sts2_c1, sts2_c2], p_target, 2, 1, 6);
    end;
    dbgIntegrals([sts2_2, sts2_3], 5);
    inc_r = sts2_3.Orbit.INC;
    lan_r = sts2_3.Orbit.LAN;
else        %this is for virtual 4-stage Shuttle
    evSTS3
    sts3_1 = flightSim3D(vehicle, 1, p_init,...
        struct('type',0, 'p',10, 'v',50, 'a',90-fs_azimuth), fs_dt);
    sts3_c1 = flightSim3D(vehicle, 1, resultsToInit(sts3_1),...
        struct('type',5, 'length',5), fs_dt);
    sts3_2 = flightSim3D(vehicle, 2, resultsToInit(sts3_c1),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    sts3_c2 = flightSim3D(vehicle, 2, resultsToInit(sts3_2),...
        struct('type',5, 'length',5), fs_dt);
    sts3_3 = flightSim3D(vehicle, 3, resultsToInit(sts3_c2),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    sts3_c3 = flightSim3D(vehicle, 3, resultsToInit(sts3_3),...
        struct('type',5, 'length',5), fs_dt);
    sts3_4 = flightSim3D(vehicle, 4, resultsToInit(sts3_c3),...
        struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
    telemetry([sts3_1, sts3_2, sts3_3, sts3_4], [sts3_c1, sts3_c2, sts3_c3], 7);
    if v3d
        trajectory([sts3_1, sts3_2, sts3_3, sts3_4], [sts3_c1, sts3_c2, sts3_c3], p_target, 2, 1, 9);
    end;
    dbgIntegrals([sts3_2, sts3_3, sts3_4], 8);
    inc_r = sts3_4.Orbit.INC;
    lan_r = sts3_4.Orbit.LAN;
end
clearvars s v3d

%POSTPROCESS
%calculate intersection angle (total plane error)
Rx=[1,0,0;0,cosd(inc_r),-sind(inc_r);0,sind(inc_r),cosd(inc_r)];
Rz=[cosd(lan_r),-sind(lan_r),0;sind(lan_r),cosd(lan_r),0;0,0,1];
reached_iy = (Rz*Rx*[0,0,-1]')';
plane_error = acosd(dot(target_iy, reached_iy))

%CLEANUP
clearvars fs_dt fs_cycle fs_coast p_init fs_target
clearvars Binertial vorbit vEarthrot vrotx vroty fs_azimuth
clearvars Rx Ry Rz target_iy p_target
clearvars inc_r lan_r reached_iy
