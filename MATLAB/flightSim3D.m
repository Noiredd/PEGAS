%flightSim3D.m
%Experiment on 3DoF simulation in Cartesian coordinates.
%Progress map:
%   [+] reconstruct world and variables
%   [+] basic data output for verification
%   [+] unpowered motion in gravity
%   [+] basic 3D visualization
%   [+] thrust
%   [+] pitch programming
%   [+] clean up vector math
%   [+] drag
%   [ ] complete data output
%   [ ] results postprocessing & plotting
%   [ ] natural gravity turn
%   [ ] compare 2D vs 3D performance
%   [ ] reconstruct PEG
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
    global mu; global g0; global R;
    global atmpressure; global atmtemperature;
    
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
    q = zeros(N,1);     %dynamic pressure [Pa]
    pitch = zeros(N,1); %pitch command log [deg] (0 - straight up)
    g_loss = 0;         %gravity d-v losses [m/s]
    d_loss = 0;         %drag d-v losses [m/s]
    %vehicle position in cartesian XYZ frame
    r = zeros(N,3);     %from Earth's center [m]
    rmag = zeros(N,1);  %magnitude [m]
    %vehicle velocity
    v = zeros(N,3);     %relative to Earth's center [m/s]
    vmag = zeros(N,1);  %magnitude [m/s]
    %reference frame matrices
    nav = zeros(3,3);   %KSP-style navball frame (radial, North, East)
    rnc = zeros(3,3);   %PEG-style tangential frame (radial, normal, circumferential)
    
    %SIMULATION INITIALIZATION
    [r(1,1),r(1,2),r(1,3)] = sph2cart(degtorad(initial.lon), degtorad(initial.lat), R+initial.alt);
    rmag(1) = norm(r(1,:));
    v(1,:) = surfSpeedInit(r(1,:));
    vmag(1) = norm(v(1,:));
    nav = getNavballFrame(r(1,:), v(1,:));
    rnc = getCircumFrame(r(1,:), v(1,:));
    
    %MAIN LOOP
    for i=2:N
        %PITCH CONTROL
        if control.type == 1
            pitch(i) = approxFromCurve(t(i-1), prog);
            yaw = 0;
        end;
        
        %PHYSICS
        %thrust/acceleration
        p = approxFromCurve((rmag(i-1)-R)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        F(i) = isp*g0*dm;
        acc(i) = F(i)/m;
        acv = acc(i)*makeVector(nav, pitch(i), yaw);
        %gravity
        G = mu*r(i-1,:)/norm(r(i-1,:))^3;           %acceleration [m/s^2]
        g_loss = g_loss + norm(G)*dt;               %integrate gravity losses
        %drag
        vair = v(i-1,:) - surfSpeed(r(i-1,:), nav); %air velocity (relative to surface)
        vairmag = norm(vair);
        if vairmag==0
            %since we later divide by this and in first iteration it's zero
            vairmag = 1;
        end;
        cd = approxFromCurve(vairmag, drag);        %drag coefficient
        temp = approxFromCurve((rmag(i-1)-R)/1000, atmtemperature)+273.15;
        dens = calculateAirDensity(p*101325, temp);
        q(i) = 0.5*dens*vairmag^2;                  %dynamic pressure
        D = area*cd*q(i)/m;                         %drag-induced acceleration [m/s^2]
        d_loss = d_loss + D*dt;                     %integrate drag losses
        %velocity
        v(i,:) = v(i-1,:) + acv*dt - G*dt - D*vair/vairmag*dt;
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        rmag(i) = norm(r(i,:));
        %local reference frames
        nav = getNavballFrame(r(i,:), v(i,:));
        rnc = getCircumFrame(r(i,:), v(i,:));
        %MASS&TIME
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
    end;
    %OUTPUT
    plots = struct('t', t(1:i),...
                   'v', v,...
                   'vv', vmag,...
                   'q', q,...
                   'r', r);
    results = struct('Altitude', (norm(r(i,:))-R)/1000,...
                     'Velocity', norm(v(i,:)),...
                   'gl', g_loss,...
                   'dl', d_loss,...
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
    %comment/uncomment line below to switch between Earth-oriented and trajectory-based plots
    %scale=scale*50;plot3(R*sx,R*sy,R*sz,'g'); scatter3(0,0,0,'g');
        %ground path
        gp=zeros(N,3);
        for i=1:N
            gp(i,:)=R*r(i,:)/norm(r(i,:));
        end
        plot3(gp(:,1),gp(:,2),gp(:,3),'k');
    %local circumferential versors
    scatter3(r(i,1),r(i,2),r(i,3),'r');
    scale=scale*50000;
    t=zeros(2,3);
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*rnc(1,:); %radial (away)
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*rnc(2,:); %normal (plane change)
    plot3(t(:,1),t(:,2),t(:,3),'g');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*rnc(3,:); %circumferential (prograde)
    plot3(t(:,1),t(:,2),t(:,3),'b');
    %navball versors
    scale = scale/2;
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nav(1,:); %away in equatorial plane
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nav(2,:); %north
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nav(3,:); %east
    plot3(t(:,1),t(:,2),t(:,3),'k');
    %acceleration vectors
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*0.03*acv;%final
    plot3(t(:,1),t(:,2),t(:,3),'y');
    hold off;
    axis equal;
end

%constructs a local reference frame, KSP-navball style
function [f] = getNavballFrame(r, v)
    %pass current position under r (1x3)
    %current velocity under v (1x3)
    pseudo_up = [r(1) r(2) 0]/norm([r(1) r(2) 0]);
    pseudo_north = cross([r(1) r(2) 0],[v(1) v(2) 0]);
    pseudo_north = pseudo_north/norm(pseudo_north);
    east = cross(pseudo_north,pseudo_up);   %true East direction
    up = r/norm(r);             %true Up direction (radial away from Earth)
    north = cross(up, east);    %true North direction (completes frame)
    f = zeros(3,3);
    %return a right-handed coordinate system base
    f(1,:) = up;
    f(2,:) = north;
    f(3,:) = east;
end

%constructs a local reference frame in style of PEG coordinate base
function [f] = getCircumFrame(r, v)
    %pass current position under r (1x3)
    %current velocity under v (1x3)
    radial = r/norm(r);             %Up direction (radial away from Earth)
    normal = cross(r, v);
    normal = normal/norm(normal);   %Normal direction (perpendicular to orbital plane)
    circum = cross(normal, radial); %Circumferential direction (tangential to sphere, in motion plane)
    f = zeros(3,3);
    %return a left(?)-handed coordinate system base
    f(1,:) = radial;
    f(2,:) = normal;
    f(3,:) = circum;
end

%finds rotation angle between the two frames
function [alpha] = rnc2nav(rnc, nav)
    %pass reference frame matrices
    %by their definitions, their 'radial' component is the same, therefore
    %rotation between them can be described with a single number
    alpha = dot(rnc(3,:), nav(3,:));
end

%constructs a unit vector in the global frame for a given azimuth/elevation
%angles in a given frame (first angle ('p') rotates from frame's first
%towards second vector, second ('y') rotates from second towards third)
function [v] = makeVector(frame, p, y)
    V = zeros(3,3);
    V(1,:) = cosd(p)*frame(1,:);
    V(2,:) = sind(p)*sind(y)*frame(2,:);
    V(3,:) = sind(p)*cosd(y)*frame(3,:);
    v = V(1,:) + V(2,:) + V(3,:);
end

%finds Earth's rotation velocity vector at given cartesian location using
%no velocity vector (meant for rotational velocity initilization)
function [rot] = surfSpeedInit(r)
%    global R;
    %create temporary frame by generating a dummy velocity vector
    t = r*[0 1 0; -1 0 0; 0 0 1];   %90 degrees counterclockwise around Z axis, looking down
    f = getNavballFrame(r, t);      %temp frame
    rot = surfSpeed(r, f);          %use a standard function
%    %get latitude
%    [~,lat,~] = cart2sph(r(1), r(2), r(3));
%    vel = 2*pi*R/(24*3600); %equatorial
%    vel = vel*cos(lat);     %at latitude
%    %now, find vector pointing in direction of Earth's rotation
%    %first rotate our location 90 degrees about Z axis (looking down - counterclockwise)
%    m = [0 1 0; -1 0 0; 0 0 1];
%    t = r*m;
%    %wind blows east - get the vector using a tool we already have
%    rot = vel*cFromNavball(r, t, 90, 0);
end

%finds Earth's rotation velocity vector at given cartesian location
%assuming a navball reference frame is available
function [rot] = surfSpeed(r, nav)
    global R;
    [~,lat,~] = cart2sph(r(1), r(2), r(3));
    vel = 2*pi*R/(24*3600)*cos(lat);
    rot = vel*nav(3,:); %third componend is East vector
end