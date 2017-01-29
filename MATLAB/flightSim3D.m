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
function [results] = flightSim3D(vehicle, stage, initial, control, dt)
    %declare globals
    global mu; global g0; global R;
    global atmpressure; global atmtemperature;
    
    %VEHICLE UNPACK
    MODE = vehicle(stage).MODE;
    m = vehicle(stage).m0;
    gLim = vehicle(stage).gLim;
    engines = vehicle(stage).engines;
    maxT = vehicle(stage).maxT;
    area = vehicle(stage).area;
    drag = vehicle(stage).drag;
    
    %CONTROL SETUP
    if control.type == 0
        %type 0 = natural gravity turn simulation
        gtiP = control.p;   %initial pitchover angle for gravity turn
        gtiV = control.v;   %velocity at which the pitchover begins
        if isfield(control, 'a')
            azim = control.a;   %azimuth, if chosen
        else
            azim = 90;
        end
        GT = 0; %gravity turn status flag:
                    %0 - not begun yet;
                    %1 - equaling to flight angle;
                    %2 - match flight angle
        ENG = -1;
    elseif control.type == 1
        %type 1 = pitch program control, constant azimuth
        prog = control.program;
        azim = control.azimuth; %for now only constant, no programming
        ENG = -1;
    elseif control.type == 2
        fprintf('Powered Explicit Guidance mode for 3D simulation is deprecated! Use UPFG instead (type 3).\n');
        return;
    elseif control.type == 3
        %type 3 = Unified Powered Flight Guidance
        target = control.target;
        ct = control.major;
        lc = 0;
        ENG = 1;    %engine state flag:
                        %0 - fuel deprived;
                        %1 - running;
                        %2 - cut as scheduled by PEG
                        %3 - cut exceptionally by a velocity limit
    elseif control.type == 5
        %type 5 = coast phase (unguided free flight)
        %strongly recommended using initial.type==1
        maxT = control.length;
        ENG = -1;
    end;
    
    %SIMULATION SETUP
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
    ang_p_srf = zeros(N,1); %flight pitch angle, surface related
    ang_y_srf = zeros(N,1); %flight yaw angle, surface related
    ang_p_obt = zeros(N,1); %flight pitch angle, orbital (absolute)
    ang_y_obt = zeros(N,1); %flight yaw angle, orbital (absolute)
    
    %SIMULATION INITIALIZATION
    if initial.type==0      %launch from static site
        %btw, wonder what would happen if one wanted to launch from the North Pole :D
        [r(1,1),r(1,2),r(1,3)] = sph2cart(degtorad(initial.lon), degtorad(initial.lat), R+initial.alt);
        nav = getNavballFrame(r(1,:));
        v(1,:) = surfSpeed(r(1,:), nav);
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
    nav = getNavballFrame(r(1,:));
    rnc = getCircumFrame(r(1,:), v(1,:));
    vair(1,:) = v(1,:) - surfSpeed(r(1,:), nav);
    vairmag(1) = max(norm(vair(1)),1);
    vy(1) = dot(v(1,:),nav(1,:));
    vt(1) = dot(v(1,:),rnc(3,:));
    ang_p_srf(1) = getAngleFromFrame(vair(1,:), nav, 'pitch');
    ang_y_srf(1) = getAngleFromFrame(vair(1,:), nav, 'yaw');
    ang_p_obt(1) = getAngleFromFrame(v(1,:), nav, 'pitch');
    ang_y_obt(1) = getAngleFromFrame(v(1,:), nav, 'yaw');
    
    %PEG SETUP
    if control.type==3
        %below 2 lines just to avoid 0 acceleration point in plots
        p = approxFromCurve((rmag(1)-R)/1000, atmpressure);
        [temp, ~, ~] = getThrust(engines, p, t(1));
        acc(1) = temp/m;
        upfg_state = struct('time', t(1), 'mass', m, 'radius', r(1,:), 'velocity', v(1,:));
        cser = struct('dtcp', 0, 'xcp', 0, 'A', 0, 'D', 0, 'E', 0);
        %guidance initialization:
        %project initial position direction unit vector onto target plane,
        %rotate with Rodrigues' formula about 20 degrees prograde and
        %extend to target length; then calculate velocity at this point
        %https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
        rdinit = unit(r(1,:));
        rdinit = rdinit - dot(rdinit,target.normal)*target.normal;
        rdinit = rdinit*cosd(20) + cross(-target.normal,rdinit)*sind(20) - target.normal*dot(-target.normal,rdinit)*(1-cosd(20));
        rdinit = rdinit * target.radius;
        vdinit = target.velocity*unit(cross(-target.normal,rdinit));
        vdinit = vdinit - v(1,:);
        %Create internal state for UPFG - continue from last stage if
        %possible. First we need to make sure there's any state before we
        %can query its genuineness.
        if isfield(initial, 'upfg')
            if isfield(initial.upfg, 'tgo')
                %We run UPFG once to obtain initial guidance values.
                upfg_internal = initial.upfg;
                upfg_internal.tb = 0;
                dbg = debugInitializator(floor(maxT/ct));
                [upfg_internal, guidance, debug] = unifiedPoweredFlightGuidance(...
                                   vehicle(stage:length(vehicle)),...
                                   target, upfg_state, upfg_internal);
                dbg = debugAggregator(dbg, debug);
            end;
        end;
        if ~exist('upfg_internal', 'var')
            %If no initial state was given, a new one must be built and
            %converged. That's also why two different initializators are
            %needed.
            upfg_internal = struct('cser', cser, 'rbias', [0,0,0], 'rd', rdinit,...
                                   'rgrav', -(mu/2)*r(1,:)/norm(r(1,:))^3,...
                                   'tb', 0, 'time', t(1), 'tgo', 0,...
                                   'v', v(1,:), 'vgo', vdinit);
            dbg = debugInitializator(floor(maxT/ct)+5);
            [upfg_internal, guidance, debug] = convergeUPFG(vehicle(stage:length(vehicle)),...
                                                            target, upfg_state, upfg_internal,...
                                                            50);
            dbg = debugAggregator(dbg, debug);
        end;
        pitch(1) = guidance.pitch;
        yaw(1) = guidance.yaw;
    end;
    
    %MAIN LOOP
    for i=2:N
        %GUIDANCE
        if control.type == 0
            %natural, lock-prograde gravity turn
            %First if-set controls current state - initial is GT==0 which
            %means vehicle is going straight up, building speed. GT==1
            %means it's going fast enough and starts pitching over in the
            %given direction OR that it already reached max allowed pitch
            %and is waiting for velocity vector to align with it. GT==2
            %means velocity vector has aligned and vehicle will now lock on
            %prograde direction.
            if dot(v(i-1,:), nav(1,:)) >= gtiV && GT == 0
                GT = 1;
            elseif (ang_p_srf(i-1) > gtiP && GT == 1)
                GT = 2;
            end;
            %Second if-set controls what to do depending on current state.
            if GT == 0
                pitch(i) = 0;
                yaw(i) = azim;
            elseif GT == 1
                %pitching over to a given angle at a hardcoded 1deg/s
                pitch(i) = min(pitch(i-1)+dt, gtiP);
                yaw(i) = azim;
            else
                pitch(i) = ang_p_srf(i-1);
                yaw(i) = azim;
            end;
        elseif control.type == 1
            %pitch program control, with possible yaw control too
            pitch(i) = approxFromCurve(t(i-1), prog);
            yaw(i) = azim;
        elseif control.type == 3
            %UPFG pitch&yaw control
            %check if there's still fuel
            if (t(i-1)-t(1) > maxT && ENG > 0)
                ENG = 0;    %engine ran out of fuel
                break;      %exit the main simulation loop
            end;
            %check how long ago was the last PEG cycle
            if (lc < ct-dt) %not too long ago - increment
                lc = lc + dt;
            else %run guidance
                upfg_state.time     = t(i-1);
                upfg_state.mass     = m;
                upfg_state.radius   = r(i-1,:);
                upfg_state.velocity = v(i-1,:);
                [upfg_internal, guidance, debug] = unifiedPoweredFlightGuidance(...
                               vehicle(stage:length(vehicle)),...
                               target, upfg_state, upfg_internal);
                dbg = debugAggregator(dbg, debug);
                %handle divergence possibility - deferred (will never
                %happen because UPFG no longer checks for that)
                if dbg.diverge(dbg.THIS) && ~dbg.diverge(dbg.THIS-1)
                    fprintf('UPFG started to diverge at t+%f\n', t(i-1));
                end
                %also make sure we don't get back to a converged state
                if ~dbg.diverge(dbg.THIS) && dbg.diverge(dbg.THIS-1)
                    dbg.diverge(dbg.THIS) = 1;
                end
                lc = 0;
            end;
            %PEG-scheduled cutoff
            if (guidance.tgo-lc < dt && ENG == 1)
                ENG = 2;
                break;
            end;
            %velocity-driven cutoff (in case UPFG went crazy, this will cut
            %off the engine when absolute target velocity is reached)
            if (norm(v(i-1,:))>=target.velocity)
                ENG = 3;
                break;
            end;
            %pitch&yaw control
            pitch(i) = guidance.pitch;
            yaw(i)   = guidance.yaw;
        end;
        
        %PHYSICS
        %Crash detection
        if rmag(i-1)<=R
            ENG = -100;
            break;
        end
        %Thrust: zero for coast flight, different calculations for constant
        %thrust and constant acceleration modes.
        p = approxFromCurve((rmag(i-1)-R)/1000, atmpressure);
        if control.type==5
            F(i) = 0;
            dm = 0;
        else
            %calculate default 100% thrust
            [F(i), dm, ~] = getThrust(engines, p, t(i-1));
            %adjust for constant acceleration if necessary
            if MODE==2
                desiredThrust = gLim*g0 * m;
                desiredThrottle = desiredThrust/F(i);
                desiredThrottle = min(desiredThrottle,engines(1).data(2));  %minimum throttle clamp
                desiredThrottle = max(desiredThrottle,engines(1).data(1));  %maximum throttle clamp
                F(i) = F(i) * desiredThrottle;
                dm = dm * desiredThrottle;
            end
        end;
        acc(i) = F(i)/m;
        acv = acc(i)*makeVector(nav, pitch(i), yaw(i));
        %gravity
        G = mu*r(i-1,:)/rmag(i-1)^3;                %acceleration [m/s^2]
        g_loss = g_loss + norm(G)*dt;               %integrate gravity losses
        %drag
        cd = approxFromCurve(vairmag(i-1), drag);   %drag coefficient
        temp = approxFromCurve((rmag(i-1)-R)/1000, atmtemperature)+273.15;
        dens = calculateAirDensity(p*101325, temp);
        q(i) = 0.5*dens*vairmag(i-1)^2;             %dynamic pressure
        D = area*cd*q(i)/m;                         %drag-induced acceleration [m/s^2]
        d_loss = d_loss + D*dt;                     %integrate drag losses
        %absolute velocities
        v(i,:) = v(i-1,:) + acv*dt - G*dt - D*unit(vair(i-1,:))*dt;
        vmag(i) = norm(v(i,:));
        vy(i) = dot(v(i,:),nav(1,:));
        vt(i) = dot(v(i,:),rnc(3,:));
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        rmag(i) = norm(r(i,:));
        %local reference frames
        nav = getNavballFrame(r(i,:));
        rnc = getCircumFrame(r(i,:), v(i,:));
        %surface velocity (must be here because needs reference frames)
        vair(i,:) = v(i,:) - surfSpeed(r(i,:), nav);
        vairmag(i) = norm(vair(i,:));
        %angles
        ang_p_srf(i) = getAngleFromFrame(vair(i,:), nav, 'pitch');
        ang_y_srf(i) = getAngleFromFrame(vair(i,:), nav, 'yaw');
        ang_p_obt(i) = getAngleFromFrame(v(i,:), nav, 'pitch');
        ang_y_obt(i) = getAngleFromFrame(v(i,:), nav, 'yaw');
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
                   'vair', vair(1:i-1,:),...
                   'vairmag', vairmag(1:i-1),...
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
                     'Plots', plots, 'ENG', ENG);
    %Handle UPFG state persistence between stages. Turns out it is CRUCIAL
    %for multistage guidance capability.
    %For guided stages, stores final UPFG internal state in the results
    %struct. For unguided ones (ie. if there is no final state), tries to
    %save the state passed to the simulation initialization struct - this
    %allows handling coasting between guided stages by resultsToInit
    %(guided stage returns UPFG state, coast stage simply copies it, next
    %guided stage continues from that state). For compatibility (in plot
    %functions especially), all result structs must have the same set of
    %fields, so in case no UPFG was ever called in a stage, a dummy is
    %created to take its place.
    if exist('upfg_internal', 'var')==1
        results().UPFG = upfg_internal;
    elseif isfield(initial, 'upfg')
        results().UPFG = initial.upfg;
    else
        results().UPFG = struct();
    end;
    [results.Apoapsis, results.Periapsis, results.Orbit.SMA,...
                    results.Orbit.ECC, results.Orbit.INC,...
                    results.Orbit.LAN, results.Orbit.AOP,...
                    results.Orbit.TAN] = getOrbitalElements(r(i-1,:), v(i-1,:));
    [results.maxQt, results.maxQv] = getMaxValue(q');   %get time and value of maxQ
    results.maxQt = t(results.maxQt);                   %format maxQ time to seconds
end

%constructs a local reference frame, KSP-navball style
function [f] = getNavballFrame(r)
    %pass current position under r (1x3)
    up = unit(r);                   %true Up direction (radial away from Earth)
    east = cross([0,0,1],up);       %true East direction
    north = cross(up, east);        %true North direction (completes frame)
    f = zeros(3,3);
    %return a right-handed coordinate system base
    f(1,:) = up;
    f(2,:) = unit(north);
    f(3,:) = unit(east);
end

%constructs a local reference frame in style of PEG coordinate base
function [f] = getCircumFrame(r, v)
    %pass current position under r (1x3)
    %current velocity under v (1x3)
    radial = unit(r);               %Up direction (radial away from Earth)
    normal = unit(cross(r, v));     %Normal direction (perpendicular to orbital plane)
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

%finds Earth's rotation velocity vector at given cartesian location
function [rot] = surfSpeed(r, nav)
    global R;
    global period;
    [~,lat,~] = cart2sph(r(1), r(2), r(3));
    vel = 2*pi*R/period * cos(lat);
    rot = vel*nav(3,:); %third componend is East vector
end

%initializes UPFG debug data aggregator with zero vectors of appropriate sizes
%pass expected length of the vector (number of guidance iterations, usually
%maxT / guidance cycle + 5 should be okay)
function [a] = debugInitializator(n)
    a = struct('THIS', 0,...
               'time', zeros(n,1),...
               'r', zeros(n,4),...
               'v', zeros(n,4),...
               'm', zeros(n,1),...
               'dvsensed', zeros(n,4),...
               'vgo1', zeros(n,4),...
               'L1', zeros(n,1),...
               'tgo', zeros(n,1),...
               'L', zeros(n,1),...
               'J', zeros(n,1),...
               'S', zeros(n,1),...
               'Q', zeros(n,1),...
               'P', zeros(n,1),...
               'H', zeros(n,1),...
               'lambda', zeros(n,4),...
               'rgrav1', zeros(n,4),...
               'rgo1', zeros(n,4),...
               'iz1', zeros(n,4),...
               'rgoxy', zeros(n,4),...
               'rgoz', zeros(n,1),...
               'rgo2', zeros(n,4),...
               'lambdade', zeros(n,1),...
               'lambdadot', zeros(n,4),...
               'iF', zeros(n,4),...
               'phi', zeros(n,1),...
               'phidot', zeros(n,1),...
               'vthrust', zeros(n,4),...
               'rthrust', zeros(n,4),...
               'vbias', zeros(n,4),...
               'rbias', zeros(n,4),...
               'pitch', zeros(n,1),...
               'EAST', zeros(n,4),...
               'yaw', zeros(n,1),...
               'rc1', zeros(n,4),...
               'vc1', zeros(n,4),...
               'rc2', zeros(n,4),...
               'vc2', zeros(n,4),...
               'cser_dtcp', zeros(n,1),...
               'cser_xcp', zeros(n,1),...
               'cser_A', zeros(n,1),...
               'cser_D', zeros(n,1),...
               'cser_E', zeros(n,1),...
               'vgrav', zeros(n,4),...
               'rgrav2', zeros(n,4),...
               'rp', zeros(n,4),...
               'rd', zeros(n,4),...
               'ix', zeros(n,4),...
               'iz2', zeros(n,4),...
               'vd', zeros(n,4),...
               'vgop', zeros(n,4),...
               'dvgo', zeros(n,4),...
               'vgo2', zeros(n,4),...
               'diverge', zeros(n,1));
end

%handles UPFG debug data aggregating
%adds debug data from a single guidance iteration into aggregated, time-based
%struct of vectors
%pass initialized debug structure and UPFG debug output
function [a] = debugAggregator(a, d)
    %we must know where to put the new results
    i = a.THIS + 1;
    a.THIS = i;
    %and onto the great copy...
    a.time(i) = d.time;
    a.r(i,1:3) = d.r;
    a.r(i,4) = norm(d.r);
    a.v(i,1:3) = d.v;
    a.v(i,4) = norm(d.v);
    a.m(i) = d.m;
    a.dvsensed(i,1:3) = d.dvsensed;
    a.dvsensed(i,4) = norm(d.dvsensed);
    a.vgo1(i,1:3) = d.vgo1;
    a.vgo1(i,4) = norm(d.vgo1);
    a.L1(i) = d.L1;
    a.tgo(i) = d.tgo;
    a.L(i) = d.L;
    a.J(i) = d.J;
    a.S(i) = d.S;
    a.Q(i) = d.Q;
    a.P(i) = d.P;
    a.H(i) = d.H;
    a.lambda(i,1:3) = d.lambda;
    a.lambda(i,4) = norm(d.lambda);
    a.rgrav1(i,1:3) = d.rgrav1;
    a.rgrav1(i,4) = norm(d.rgrav1);
    a.rgo1(i,1:3) = d.rgo1;
    a.rgo1(i,4) = norm(d.rgo1);
    a.iz1(i,1:3) = d.iz1;
    a.iz1(i,4) = norm(d.iz1);
    a.rgoxy(i,1:3) = d.rgoxy;
    a.rgoxy(i,4) = norm(d.rgoxy);
    a.rgoz(i) = d.rgoz;
    a.rgo2(i,1:3) = d.rgo2;
    a.rgo2(i,4) = norm(d.rgo2);
    a.lambdade(i) = d.lambdade;
    a.lambdadot(i,1:3) = d.lambdadot;
    a.lambdadot(i,4) = norm(d.lambdadot);
    a.iF(i,1:3) = d.iF;
    a.iF(i,4) = norm(d.iF);
    a.phi(i) = d.phi;
    a.phidot(i) = d.phidot;
    a.vthrust(i,1:3) = d.vthrust;
    a.vthrust(i,4) = norm(d.vthrust);
    a.rthrust(i,1:3) = d.rthrust;
    a.rthrust(i,4) = norm(d.rthrust);
    a.vbias(i,1:3) = d.vbias;
    a.vbias(i,4) = norm(d.vbias);
    a.rbias(i,1:3) = d.rbias;
    a.rbias(i,4) = norm(d.rbias);
    a.pitch(i) = d.pitch;
    a.EAST(i,1:3) = d.EAST;
    a.EAST(i,4) = norm(d.EAST);
    a.yaw(i) = d.yaw;
    a.rc1(i,1:3) = d.rc1;
    a.rc1(i,4) = norm(d.rc1);
    a.vc1(i,1:3) = d.vc1;
    a.vc1(i,4) = norm(d.vc1);
    a.rc2(i,1:3) = d.rc2;
    a.rc2(i,4) = norm(d.rc2);
    a.vc2(i,1:3) = d.vc2;
    a.vc2(i,4) = norm(d.vc2);
    a.cser_dtcp(i) = d.cser.dtcp;
    a.cser_xcp(i) = d.cser.xcp;
    a.cser_A(i) = d.cser.A;
    a.cser_D(i) = d.cser.D;
    a.cser_E(i) = d.cser.E;
    a.vgrav(i,1:3) = d.vgrav;
    a.vgrav(i,4) = norm(d.vgrav);
    a.rgrav2(i,1:3) = d.rgrav2;
    a.rgrav2(i,4) = norm(d.rgrav2);
    a.rp(i,1:3) = d.rp;
    a.rp(i,4) = norm(d.rp);
    a.rd(i,1:3) = d.rd;
    a.rd(i,4) = norm(d.rd);
    a.ix(i,1:3) = d.ix;
    a.ix(i,4) = norm(d.ix);
    a.iz2(i,1:3) = d.iz2;
    a.iz2(i,4) = norm(d.iz2);
    a.vd(i,1:3) = d.vd;
    a.vd(i,4) = norm(d.vd);
    a.vgop(i,1:3) = d.vgop;
    a.vgop(i,4) = norm(d.vgop);
    a.dvgo(i,1:3) = d.dvgo;
    a.dvgo(i,4) = norm(d.dvgo);
    a.vgo2(i,1:3) = d.vgo2;
    a.vgo2(i,4) = norm(d.vgo2);
    a.diverge(i) = d.diverge;
end

%handles UPFG convergence by running it in loop until tgo stabilizes
function [internal, guidance, debug] = convergeUPFG(vehicle, target, state, internal, maxIters)
    global convergenceCriterion;
    fail = 1;
    [internal, guidance, debug] = unifiedPoweredFlightGuidance(vehicle, target, state, internal);
    for i=1:maxIters
        t1 = internal.tgo;
        [internal, guidance, debug] = unifiedPoweredFlightGuidance(vehicle, target, state, internal);
        t2 = internal.tgo;
        if abs( (t1-t2)/t1 ) < convergenceCriterion
            fprintf('UPFG converged after %d iterations (tgo=%d).\n', i, t2);
            fail = 0;
            break;
        end
    end
    if fail
        fprintf('UPFG failed to converge in %d iterations!\n', i);
    end
end