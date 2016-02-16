%ascentSimulation.m
%2DOF atmospheric ascent simulation. Supports pitch control via time table
%as well as pure gravity turn (lock on prograde). Pass control parameters
%in 'control' struct (see section 'define control type' for details). Pass
%simulation step time in 'dt'.
%Vehicle is modelled as a point mass with drag. Simulation is located in a
%rotating frame of reference, so a centrifugal force appears. Vehicle is
%assumed to only have one engine and fuel tank. RO atmosphere (pressure and
%temperature) is modelled to calculate accurate air density. No AoA or lift
%effects are taken into account.
%Dependencies:
%   approxFromCurve.m
%   calculateAirDensity.m
%   getMaxValue.m
%   getOrbital.m
function [results] = ascentSimulation(control, dt)
    %declare globals
    global mu; global g0; global R; global VEHICLE1;
    global atmpressure;  global atmtemperature;
    
    %unpack vehicle data
    m = VEHICLE1.m0;
    isp0 = VEHICLE1.i0;
    isp1 = VEHICLE1.i1;
    dm = VEHICLE1.dm;
    maxT = VEHICLE1.mt;
    engT = VEHICLE1.et;
    A = VEHICLE1.ra;
    dragcurve = VEHICLE1.dc;
    
    %define control type
    if control.type == 0
        %type 0 = natural gravity turn simulation
        gtiP = control.p;   %initial pitchover angle for gravity turn
        gtiV = control.v;   %velocity at which the pitchover begins
        GT = 0; %gravity turn status flag: 0 - not begun yet; 1 - equaling to flight angle; 2 - match flight angle
    elseif control.type == 1
        %type 1 = pitch program control
        prog = control.program;
    end
    
    %SIMULATION SETUP
    %simulate engine ignition effects
    m = m - engT*dm;    %rocket is burning fuel while bolted to the launchpad for engT seconds before it's released
    maxT = maxT - engT; %this results in loss of propellant mass and hence reduction of maximum burn time
    N = floor(maxT/dt)+1;%simulation steps (precision supplied by argument)
    t = zeros(1,N);     %current time
    C = 0;              %centrifugal acceleration [m/s^2] (we use a cartesian non rotating frame of reference)
    F = zeros(1,N);     %thrust [N]
    q = zeros(1,N);     %dynamic pressure q [Pa]
    D = zeros(1,N);     %drag [N]
    G = 0;              %current gravity acceleration [m/s^2]
    vx = zeros(1,N);    %horizontal (tangential) velocity (towards the orbital velocity) [m/s]
    vx_gain = (2*pi*R/24/3600)*...
     cosd(VEHICLE1.lat);%gained from Earth's rotational motion
    vy = zeros(1,N);    %vertical (radial) velocity (altitude change) [m/s]
    angle = zeros(1,N); %velocity vector direction log [deg] (0 - straight up, 90 - due east)
    pitch = zeros(1,N); %vehicle orientation log [deg] (pitch commands) (0 - straight up, 90 - due east)
    vg = 0;             %gravity losses (integrated) [m/s]
    vd = 0;             %aerodynamic losses (integrated) [m/s]
    alt = zeros(1,N);   %altitude (integral) [m] (measured from the surface)
    alt(1) = VEHICLE1.lsa;
    rad = zeros(1,N);   %radial distance from launch point (eg. geographical longitude, for a GTO launch) [deg]
    
    %MAIN LOOP
    for i=2:N
        %PITCH CONTROL SECTION
        if control.type == 0
            %natural, lock-prograde gravity turn
            if vy(i-1) >= gtiV && GT == 0
                GT = 1;
            elseif angle(i-1) > gtiP && GT == 1
                GT = 2;
            end;
            if GT == 0
                pitch(i) = 0;
            elseif GT == 1
                pitch(i) = min(pitch(i-1)+dt, gtiP);    %hardcoded 1deg/s change (to simulate real, not instantaneous pitchover)
            else
                pitch(i) = angle(i-1);
            end;
        elseif control.type == 1
            %pitch program control
            pitch(i) = approxFromCurve(t(i-1), prog);
        end
        
        %PHYSICS SIMULATION PART
        %get isp
        p = approxFromCurve(alt(i-1)/1000, atmpressure); %get pressure
        isp = (isp1-isp0)*p+isp0;
        %calculate thrust
        F(i) = isp*g0*dm;
        %new velocity
        dv = F(i)/m * dt;
        vx(i) = vx(i-1) + dv*sind(pitch(i-1));
        vy(i) = vy(i-1) + dv*cosd(pitch(i-1));
        %centrifugal and gravitational acceleration
        C = vx(i-1)^2/(alt(i-1)+R);
        G = mu/(R+alt(i-1))^2;
        vy(i) = vy(i) + (C - G) * dt;
        vg = vg + G * dt;%integrate gravity losses
        %aerodynamic losses (simplest way: no lift or AoA effects)
        airv = vy(i-1)^2 + vx(i-1)^2;  %should be sqrt of that, but we need squared value at q
        cd = approxFromCurve(sqrt(airv), dragcurve); %vehicle drag coefficient at given velocity
        temp = approxFromCurve(alt(i-1)/1000, atmtemperature)+273.15; %air temperature at altitude
        dens = calculateAirDensity(p*101325, temp);
        q(i) = 0.5*dens*airv;  %dynamic pressure
        D(i) = A*cd*q(i);  %aerodynamic drag force (original formula: 0.5*A*cd*dens*airv*airv)
        dv = D(i)/m * dt;  %reuse an old variable, this time for a velocity decrement due to drag
        vx(i) = vx(i) - dv*sind(angle(i-1));
        vy(i) = vy(i) - dv*cosd(angle(i-1));
        vd = vd + dv;%integrate aerodynamic losses
        %velocity vector angle
        angle(i) = asind(vx(i) / sqrt(vx(i)^2+vy(i)^2));
        %integrate altitude and radial distance
        alt(i) = alt(i-1) + vy(i)*dt;
        rad(i) = rad(i-1) + atan2d(vx(i)*dt, alt(i)+R);
        %update mass and time
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
        %legacy crash check
        if alt(i) < 0
            break;
        end;
    end
    %prepare summary
    plots = struct('t', t,...
                   'F', F,...
                   'q', q,...
                   'D', D,...
                   'vx', vx,...
                   'vy', vy,...
                   'angle', angle,...
                   'pitch', pitch,...
                   'altitude', alt,...
                   'radial', rad);
    airv = sqrt((vx(N)+vx_gain)^2+vy(N)^2);
    results = struct('Altitude', alt(N)/1000,...
                     'Velocity', airv,...
                     'Angle', acosd(vy(N)/airv),...
                     'Apoapsis', 0, 'maxQv', 0, 'maxQt', 0,...
                     'LostGravity', vg,...
                     'LostDrag', vd,...
                     'LostTotal', vg+vd,...
                     'Plots', plots);
    [results.Apoapsis,~] = getOrbital(vx(N)+vx_gain, vy(N), alt(N)+R); %returns apoapsis and periapsis
    [results.maxQt, results.maxQv] = getMaxValue(q); %returns maxQ's occurence (index in time table) and value
    results.maxQt = t(results.maxQt); %format to seconds