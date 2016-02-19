%flightSim3D.m
%Experiment on 3DoF simulation in Cartesian coordinates.
%Progress map:
%   [+] reconstruct world and variables
%   [+] basic data output for verification
%   [+] unpowered motion in gravity
%   [+] basic 3D visualization
%   [+] thrust
%   [+] pitch programming
%   [ ] drag
%   [ ] complete data output
%   [ ] results postprocessing & plotting
%   [ ] natural gravity turn
%   [ ] compare 2D vs 3D performance
%   [ ] reconstruct PEG
%   [ ] clean up vector math (until this point we'll be doing everything multiple times)
%   [ ] orbital elements from r&v
%   [ ] passive yaw programming (verify azimuth~inclination results)
%   [ ] PEG YAW CONTROL
%   [ ] ...
%   [ ] sample flight comparison (MATLAB vs kOS, constant azimuth)
%       examine how inclination&LAN behave for purposes of launch timing
%   [ ] ...
%   [ ] MULTISTAGE
function [results] = flightSim3D(vehicle, initial, control, dt)
    %declare globals
    global mu, global g0, global R;
    global atmpressure;
    
    %VEHICLE UNPACK
    m = vehicle.m0;
    isp0 = vehicle.i0;
    isp1 = vehicle.i1;
    dm = vehicle.dm;
    maxT = vehicle.mt;
    engT = vehicle.et;
    area = vehicle.ra;
    drag = vehicle.dc;
    
    %CONTROL SETUP
    if control.type == 0
        return; %we're not ready yet
    elseif control.type == 1
        prog = control.program;
        %turns out it's easier to implement this than aero turn
    end;
    
    %SIMULATION SETUP
    m = m - engT*dm;    %rocket is burning fuel while bolted to the launchpad for engT seconds before it's released
    maxT = maxT - engT; %this results in loss of propellant mass and hence reduction of maximum burn time
    N = floor(maxT/dt)+1;%simulation steps
    t = zeros(N,1);     %simulation time
    F = zeros(N,1);     %thrust magnitude [N]
    acc = zeros(N,1);   %acceleration due to thrust magnitude [m/s^2]
    pitch = zeros(N,1); %pitch command log [deg] (0 - straight up)
    %vehicle position in cartesian XYZ frame
    r = zeros(N,3);     %from Earth's center [m]
    rr = zeros(N,1);    %magnitude [m]
    %vehicle velocity
    v = zeros(N,3);     %relative to Earth's center [m/s]
    vv = zeros(N,1);    %magnitude [m/s]
    %flight angles (THIS IS PROBABLY INCORRECT FOR NOW)
    fpa_srf = zeros(1,N);   %Flight Pitch Angle (surface) - how much the vehicle points "up" in relation to moving air [deg]
                            %at launchpad this is undefined. 0 = straight up, 90 = due East
    fpa_obt = zeros(1,N);   %the same but related to orbital velocity [deg]le points "up" in relation to moving air [deg]
                            %at launchpad this equals 90 (only Earth's rotation)
    fya_srf = zeros(1,N);   %Flight Yaw Angle (surface) - how much the vehicle points "South" in relation to moving air [deg]le points "up" in relation to moving air [deg]
                            %at launchpad this is undefined (stationary related to surface/air)
                            %0 = due East, 90 = due South
    fya_obt = zeros(1,N);   %the same but related to orbital velocity [deg]
                            %at launchpad this equals 90 (craft does not move towards the equator)
                            %0 = due East, 90 = due South
    %forces
    Ga = [0 0 0];           %current gravitational acceleration vector [m/s^2]
    
    %SIMULATION INITIALIZATION
    [r(1,1),r(1,2),r(1,3)] = sph2cart(degtorad(initial.lon), degtorad(initial.lat), R+initial.alt);
    rr(1) = norm(r(1,:));
    v(1,:) = surfSpeed(r(1,:));
    vv(1) = norm(v(1,:));
    ru = r(1,:)/rr(1);
    nu = cross(r(1,:),v(1,:)); nu = nu/norm(nu);
    cu = cross(nu,ru);
    un = [r(1,1) r(1,2) 0]; un = un/norm(un);
    nn = cross([r(1,1) r(1,2) 0],[v(1,1) v(1,2) 0]); nn = nn/norm(nn);
    en = cross(nn,un);
    
    temp=zeros(1,N);
    %MAIN LOOP
    for i=2:N
        %PITCH CONTROL
        if control.type == 1
            pitch(i) = approxFromCurve(t(i-1), prog);
            yaw = 0;
        end;
        
        %PHYSICS
        %thrust/acceleration
        p = approxFromCurve((rr(i-1)-R)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        F(i) = isp*g0*dm;
        acc(i) = F(i)/m;
        acv = acc(i)*cFromNavball(r(i-1,:), v(i-1,:), pitch(i), yaw);
        %%angle between planar component of thrust and east +angle between coordinate systems in plane
        %acosd(dot(acv2+acv3,en)/(norm(acv2+acv3)*norm(en)))+acosd(dot(cu,en));
        %gravity
        Ga = mu*r(i-1,:)/norm(r(i-1,:))^3;
        %velocity
        v(i,:) = v(i-1,:) - Ga*dt + acv*dt;
        vv(i) = norm(v(i,:));
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        rr(i) = norm(r(i,:));
        %vectors
            %local coordinates (as used by PEG)
        ru = r(i,:)/norm(r(i,:));   %radial versor -> our local "Up" (away from the Earth)
        nu = cross(r(i,:),v(i,:));
        nu = nu/norm(nu);           %normal versor -> our local "Left"
        cu = cross(nu,ru);          %circuferential versor -> our local "Forward" (towards the horizon)
            %"navball" coordinates: not actually what KSP navball looks
            %like - the real one would use 'ru', 'en' and cross(ru,en)
        un = [r(i,1) r(i,2) 0];
        un = un/norm(un);           %local "Equatorial Up"
        nn = cross([r(i,1) r(i,2) 0],[v(i,1) v(i,2) 0]);
        nn = nn/norm(nn);           %local "North"
        en = cross(nn,un);          %local "East"
            %angles
            temp(i)=acosd(dot(r(i,:),acv)/(norm(r(i,:))*norm(acv)));
        %MASS&TIME
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
    end;
    
    %OUTPUT
    plots = struct('t', t(1:i),...
                   'v', v,...
                   'vv', vv,...
                   'r', r);
    results = struct('Altitude', (norm(r(i,:))-R)/1000,...
                     'Velocity', norm(v(i,:)),...
                     'Plots', plots);
    %figure(1); clf; plot(temp);
    figure(2); clf;
    scale=1;
    hold on;
    plot3(r(:,1),r(:,2),r(:,3));
    [sx,sy,sz]=sphere(20);%make half a sphere
    sx=sx(11:end,:);
    sy=sy(11:end,:);
    sz=sz(11:end,:);
    %comment/uncomment line below to switch between Earth-based plot and trajectory oriented one
    %scale=scale*50;plot3(R*sx,R*sy,R*sz,'g'); scatter3(0,0,0,'g');
        %lets try a ground path instead a sphere
        gp=zeros(N,3);
        for i=1:N
            gp(i,:)=R*r(i,:)/norm(r(i,:));
        end
        plot3(gp(:,1),gp(:,2),gp(:,3),'k');
    %local circumferential versors
    scatter3(r(i,1),r(i,2),r(i,3),'r');
    scale=scale*50000;
    t=zeros(2,3);
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*ru; %radial (away)
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nu; %normal (plane change)
    plot3(t(:,1),t(:,2),t(:,3),'g');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*cu; %circumferential (prograde)
    plot3(t(:,1),t(:,2),t(:,3),'b');
        %[dot(ru,nu) dot(ru,cu) dot(nu,cu)]
    %navball versors
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*un; %away in equatorial plane
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nn; %north
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*en; %east
    plot3(t(:,1),t(:,2),t(:,3),'k');
        %[dot(un,nn) dot(un,en) dot(nn,en)]
    %acceleration
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*0.03*acv;%final
    plot3(t(:,1),t(:,2),t(:,3),'y');
    hold off;
end

%constructs a cartesian unit vector for given navball heading
function [c] = cFromNavball(r, v, p, y)
    %pass current position under r (1x3)
    %current velocity under v (1x3)
    %pitch aim under p (scalar) - degrees away from Up
    %yaw aim under y (scalar) - degrees from East towards North
    eup = [r(1) r(2) 0];
    eup = eup/norm(eup);        %local "Equatorial Up"
    north = cross([r(1) r(2) 0],[v(1) v(2) 0]);
    north = north/norm(north);  %local "North"
    east = cross(north,eup);    %true "East"
    up = r/norm(r);             %true "Up" (away from the planet)
    tn = cross(up,east);        %true "North"
    %up, north and east components
    a = cosd(p)*up;
    b = sind(p)*sind(y)*tn;
    c = sind(p)*cosd(y)*east;
    c = a + b + c;
end

%finds Earth's rotation velocity vector at given cartesian location
function [rot] = surfSpeed(r)
    global R;
    %get latitude
    [~,lat,~] = cart2sph(r(1), r(2), r(3));
    vel = 2*pi*R/(24*3600); %equatorial
    vel = vel*cos(lat);     %at latitude
    %now, find vector pointing in direction of Earth's rotation
    %first rotate our location 90 degrees about Z axis
    m = [0 1 0; -1 0 0; 0 0 1];
    t = r*m;
    %wind blows east - get the vector using a tool we already have
    rot = vel*cFromNavball(r, t, 90, 0);
end