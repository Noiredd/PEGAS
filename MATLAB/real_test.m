%SIMULATION SETUP
fs_dt = 0.25;
fs_cycle = 2;       %PEG cycle length [s]
fs_coast = 25;      %coast between stages

%INITIAL CONDITIONS
p_init = s1_init;
    %Kourou
    %p_init.lat = 5.22167;
    %p_init.lon = -52.75389;
%KSC
p_init.lat = 28.52406;
p_init.lon = -80.65085;
p_init.alt = 0;

%TARGET ORBIT SETUP
%altitude [km ASL]
fs_target = 200;
%inclination (ISS)
inc = 51.65;
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
lan = p_init.lon - asind(min(1,tand(90-inc)*tand(p_init.lat))) + 3;
if lan<360
    lan=lan+360;
end
%To calculate orbital plane normal vector, we rotate a unit vector pointing
%perfect South (negative Z axis - btw please don't ask me why, this just
%works while positive didn't). First about the X axis (corresponding to
%zero longitude meridian) by inclination angle, and then about the Z axis.
Rx=[1,0,0;0,cosd(inc),-sind(inc);0,sind(inc),cosd(inc)];    %about x for inclination (preserve zero node)
%Ry=[cosd(inc),0,sind(inc);0,1,0;-sind(inc),0,cosd(inc)];   %template if we needed it for anything
Rz=[cosd(lan),-sind(lan),0;sind(lan),cosd(lan),0;0,0,1];    %about z for node
target_iy = (Rz*Rx*[0,0,-1]')';
p_target = struct('radius', R+fs_target*1000, 'normal', target_iy,...
                  'angle', 0, 'velocity', sqrt(mu/(R+fs_target*1000)));

%SIMULATION
if 1
p_stage1 = flightSim3D(s1_vehicle, p_init,...
    struct('type',1, 'program',s1_prog, 'azimuth',fs_azimuth), fs_dt);
p_coast1 = flightSim3D(s2_vehicle, resultsToInit(p_stage1),...
    struct('type',5, 'length',fs_coast), fs_dt);
p_stage2 = flightSim3D(s2_vehicle, resultsToInit(p_coast1),...
    struct('type',3, 'target',p_target, 'major',fs_cycle), fs_dt);
%p_coast2 = flightSim3D(s2_vehicle, resultsToInit(p_stage2),...
%    struct('type',5, 'length',3000), 1);
end

%POSTPROCESS
telemetry([p_stage1, p_stage2], [p_coast1], 1);
trajectory([p_stage1, p_stage2], [p_coast1], 2, 1, 2);
%For some showoff, draw target orbit ground track and normal vector. Maybe
%incorporate it into trajectory.m?
figure(2);hold on;t=zeros(2,3);t(2,:)=target_iy*R*1.25;
tt=1:1:360;
ttt=zeros(length(tt),3);
for i=1:length(ttt)
ttt(i,:) = (R+100)*(Rz*Rx*[sind(tt(i));cosd(tt(i));0])';
end;
plot3(t(:,1),t(:,2),t(:,3),'m');
plot3(ttt(:,1),ttt(:,2),ttt(:,3),'m');
hold off;

%CLEANUP
clearvars fs_dt fs_cycle fs_coast p_init fs_target
clearvars Binertial vorbit vEarthrot vrotx vroty fs_azimuth
clearvars Rx Ry Rz target_iy p_target
clearvars t tt ttt