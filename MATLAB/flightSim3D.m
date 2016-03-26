%flightSim3D.m
%Complete 3DoF flight simulation in Cartesian coordinates.
%Pass vehicle parameters in 'vehicle' struct. Pass initial conditions in
%'init' struct (see section 'simulation initialization' for details, true
%documentation is TODO for now). Pass control method in 'control' struct
%(see 'control setup' section for details) - supports a natural gravity
%turn simulation, predefined pitch program, PEG pitch guidance and unguided
%free flight. NO yaw steering.
%Vehicle is modelled as a point mass with drag. Simulation is located in an
%Earth-centered, inertial frame of reference, so in launch simulations the
%vehicle does not begin stationary (unless on a pole). No vehicle details
%are assumed, engine is modelled only using thrust and Isp with no regard
%for actual number or type of engines. RO atmosphere is modelled. No AoA or
%lift effects are taken into account.
%Progress map:
%   [+] SIMULATION
%   [ ] PEG YAW CONTROL
%   [ ] LAN+INC TARGETING
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
        %type 0 = natural gravity turn simulation
        gtiP = control.p;   %initial pitchover angle for gravity turn
        gtiV = control.v;   %velocity at which the pitchover begins
        GT = 0; %gravity turn status flag:
                    %0 - not begun yet;
                    %1 - equaling to flight angle;
                    %2 - match flight angle
    elseif control.type == 1
        %type 1 = pitch program control, constant azimuth
        prog = control.program;
        azim = control.azimuth; %for now only constant, no programming
    elseif control.type == 2
        %type 2 = powered explicit guidance
        target = control.target*1000+R; %target orbit altitude
        azim = control.azimuth;         %YAW CONTROL COMING SOON
        ct = control.major;             %length of the major loop
        lc = 0;                         %time since last PEG cycle
        ENG = 1;                        %engine state flag:
                                            %0 - fuel deprived;
                                            %1 - running;
                                            %2 - cut as scheduled by PEG
    elseif control.type == 3
        %type 3 = coast phase (unguided free flight)
        %strongly recommended using initial.type==1
        engT = 0;
        maxT = control.length;
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
    yaw = zeros(N,1);   %yaw command log [deg] (0 - straight East, 90 - North)
    g_loss = 0;         %gravity d-v losses [m/s]
    d_loss = 0;         %drag d-v losses [m/s]
    %vehicle position in cartesian XYZ frame
    r = zeros(N,3);     %from Earth's center [m]
    rmag = zeros(N,1);  %magnitude [m]
    %vehicle velocity
    v = zeros(N,3);     %relative to Earth's center [m/s]
    vmag = zeros(N,1);  %magnitude [m/s]
    vy = zeros(N,1);    %magnitude - altitude change [m/s]
    vt = zeros(N,1);    %magnitude - tangential [m/s]
    vair = zeros(N,3);  %relavite to surface [m/s]
    vairmag = zeros(N,1);%magnitude relative to surface [m/s]
    %reference frame matrices
    nav = zeros(3,3);   %KSP-style navball frame (radial, North, East)
    rnc = zeros(3,3);   %PEG-style tangential frame (radial, normal, circumferential)
    %flight angles
    ang_p_srf = zeros(1,N); %flight pitch angle, surface related
    ang_y_srf = zeros(1,N); %flight yaw angle, surface related
    ang_p_obt = zeros(1,N); %flight pitch angle, orbital (absolute)
    ang_y_obt = zeros(1,N); %flight yaw angle, orbital (absolute)
    
    %SIMULATION INITIALIZATION
    if initial.type==0      %launch from static site
        %btw, wonder what would happen if one wanted to launch from the North Pole :D
        [r(1,1),r(1,2),r(1,3)] = sph2cart(degtorad(initial.lon), degtorad(initial.lat), R+initial.alt);
        v(1,:) = surfSpeedInit(r(1,:));
    elseif initial.type==1  %vehicle already in flight
        t(1) = initial.t;
        r(1,:) = initial.r;
        v(1,:) = initial.v;
    else
        disp('Wrong initial conditions!');
        return;
    end;
    rmag(1) = norm(r(1,:));
    vmag(1) = norm(v(1,:));
    nav = getNavballFrame(r(1,:), v(1,:));
    rnc = getCircumFrame(r(1,:), v(1,:));
    vair(1,:) = v(1,:) - surfSpeed(r(1,:), nav);
    vairmag(1) = max(norm(vair(1)),1);
    vy(1) = dot(v(1,:),nav(1,:));
    vt(1) = dot(v(1,:),rnc(3,:));
    ang_p_srf(1) = acosd(dot(vair(1,:),nav(1,:))/vairmag(1));
    ang_y_srf(1) = acosd(dot(vair(1,:),nav(3,:))/vairmag(1));
    ang_p_obt(1) = acosd(dot(v(1,:),nav(1,:))/vmag(1));
    ang_y_obt(1) = acosd(dot(v(1,:),nav(3,:))/vmag(1));
    
    %PEG SETUP (will be here)
    if control.type==2
        dbg = zeros(N,4);   %debug log (A, B, C, T)
        p = approxFromCurve((rmag(1)-R)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        ve = isp*g0;
        acc(1) = ve*dm/m;
        [A, B, C, T] = poweredExplicitGuidance(...
                        0,...
                        rmag(1), vt(1), vy(1), target,...
                        acc(1), ve, 0, 0, maxT);
        dbg(1,:) = [A, B, C, T];
        pitch(1) = acosd(A + C);
        yaw(1) = 90-azim;
    end;
    
    %MAIN LOOP
    for i=2:N
        %PITCH CONTROL
        if control.type == 0
            %natural, lock-prograde gravity turn
            %state control
            if dot(v(i-1,:), nav(1,:)) >= gtiV && GT == 0
                %vertical velocity condition matched
                GT = 1;
            elseif ang_p_srf(i-1) > gtiP && GT == 1
                %(surface) initial pitch angle reached
                GT = 2;
            end;
            %pitch control depending on state
            if GT == 0
                %vertical flight, velocity buildup
                pitch(i) = 0;
            elseif GT == 1
                %pitching over to a given angle
                pitch(i) = min(pitch(i-1)+dt, gtiP);    %hardcoded 1deg/s change (to simulate real, not instantaneous pitchover)
            else
                %pitch angle matching airspeed (thrust prograde)
                pitch(i) = ang_p_srf(i-1);
            end;
        elseif control.type == 1
            %pitch program control, with possible yaw control too
            pitch(i) = approxFromCurve(t(i-1), prog);
            yaw(i) = 90-azim;
        elseif control.type == 2
            %PEG pitch control
            %check if there's still fuel
            if (t(i)-t(1) > maxT && ENG > 0)
                ENG = 0;    %engine ran out of fuel
                break;      %exit the main simulation loop
            end;
            %check how long ago was the last PEG cycle
            if (lc < ct)
                %if not too long ago - increment
                lc = lc + dt;
            else
                %run PEG
                [A, B, C, T] = poweredExplicitGuidance(...
                                0,...
                                rmag(i-1), vt(i-1), vy(i-1), target,...
                                acc(i-1), ve, A, B, T); %passing old T instead of T-dt IS CORRECT
                lc = 0; %TODO: bypass resetting this one if PEG skipped AB recalculation
            end;
            %PEG debug logs
            dbg(i,1) = A;
            dbg(i,2) = B;
            dbg(i,3) = C;
            dbg(i,4) = T;
            %PEG-scheduled cutoff
            if (T-lc < dt && ENG == 1)
                ENG = 2;
                break;
            end;
            %pitch control (clamped to acosd domain which should never be necessary)
            pitch(i) = acosd( min(1, max(-1, A - B*lc + C)) );
            yaw(i) = 90-azim;
        end;
        
        %PHYSICS
        %thrust/acceleration
        p = approxFromCurve((rmag(i-1)-R)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        %enable coast flight
        if control.type==3
            F(i) = 0;
        else
            F(i) = isp*g0*dm;
        end;
        acc(i) = F(i)/m;
        acv = acc(i)*makeVector(nav, pitch(i), yaw(i));
        %gravity
        G = mu*r(i-1,:)/norm(r(i-1,:))^3;           %acceleration [m/s^2]
        g_loss = g_loss + norm(G)*dt;               %integrate gravity losses
        %drag
        cd = approxFromCurve(vairmag(i-1), drag);   %drag coefficient
        temp = approxFromCurve((rmag(i-1)-R)/1000, atmtemperature)+273.15;
        dens = calculateAirDensity(p*101325, temp);
        q(i) = 0.5*dens*vairmag(i-1)^2;             %dynamic pressure
        D = area*cd*q(i)/m;                         %drag-induced acceleration [m/s^2]
        d_loss = d_loss + D*dt;                     %integrate drag losses
        %absolute velocities
        v(i,:) = v(i-1,:) + acv*dt - G*dt - D*vair(i-1,:)/vairmag(i-1)*dt;
        vmag(i) = norm(v(i,:));
        vy(i) = dot(v(i,:),nav(1,:));
        vt(i) = dot(v(i,:),rnc(3,:));
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        rmag(i) = norm(r(i,:));
        %local reference frames
        nav = getNavballFrame(r(i,:), v(i,:));
        rnc = getCircumFrame(r(i,:), v(i,:));
        %surface velocity (must be here because needs reference frames)
        vair(i,:) = v(i,:) - surfSpeed(r(i,:), nav);
        vairmag(i) = norm(vair(i,:));
        if vairmag(i)==0    %since we later divide by this and in first iteration it can be zero
            vairmag(i) = 1;
        end;
        %angles
        ang_p_srf(i) = acosd(dot(vair(i,:),nav(1,:))/vairmag(i));
        ang_y_srf(i) = acosd(dot(vair(i,:),nav(3,:))/vairmag(i));
        ang_p_obt(i) = acosd(dot(v(i,:),nav(1,:))/vmag(i));
        ang_y_obt(i) = acosd(dot(v(i,:),nav(3,:))/vmag(i));
        %MASS&TIME
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
    end;
    %OUTPUT
    plots = struct('t', t(1:i-1),...
                   'r', r(1:i-1,:),...
                   'rmag', rmag(1:i-1),...
                   'v', v(1:i-1,:),...
                   'vy', vy(1:i-1),...
                   'vt', vt(1:i-1),...
                   'vmag', vmag(1:i-1),...
                   'F', F(1:i-1),...
                   'a', acc(1:i-1),...
                   'q', q(1:i-1),...
                   'pitch', pitch(1:i-1),...
                   'yaw', yaw(1:i-1),...
                   'angle_ps', ang_p_srf(1:i-1),...
                   'angle_ys', ang_y_srf(1:i-1),...
                   'angle_po', ang_p_obt(1:i-1),...
                   'angle_yo', ang_y_obt(1:i-1));
    %add debug data if created
    if exist('dbg', 'var')==1
        plots().DEBUG = dbg;
    end;
    orbit = struct('SMA', 0, 'ECC', 0, 'INC', 0,...
                   'LAN', 0, 'AOP', 0, 'TAN', 0);
    results = struct('Altitude', (rmag(i-1)-R)/1000,...
                     'Apoapsis', 0, 'Periapsis', 0,...
                     'Orbit', orbit,...
                     'Velocity', vmag(i-1),...
                     'VelocityY', dot(v(i-1,:), nav(1,:)),...
                     'VelocityT', dot(v(i-1,:), rnc(3,:)),...
                     'maxQv', 0, 'maxQt', 0,...
                     'LostGravity', g_loss,...
                     'LostDrag', d_loss,...
                     'LostTotal', g_loss+d_loss,...
                     'BurnTimeLeft', maxT-t(i-1)+t(1),...
                     'Plots', plots);
    [results.Apoapsis, results.Periapsis, results.Orbit.SMA,...
                    results.Orbit.ECC, results.Orbit.INC,...
                    results.Orbit.LAN, results.Orbit.AOP,...
                    results.Orbit.TAN] = getOrbitalElements(r(i-1,:), v(i-1,:));
    [results.maxQt, results.maxQv] = getMaxValue(q');   %get time and value of maxQ
    results.maxQt = t(results.maxQt);                   %format maxQ time to seconds
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
end

%finds Earth's rotation velocity vector at given cartesian location
%assuming a navball reference frame is available
function [rot] = surfSpeed(r, nav)
    global R;
    [~,lat,~] = cart2sph(r(1), r(2), r(3));
    vel = 2*pi*R/(24*3600)*cos(lat);
    rot = vel*nav(3,:); %third componend is East vector
end