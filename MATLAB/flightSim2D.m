function [results] = flightSim2D(vehicle, initial, control, dt)
%results = FLIGHTSIM2D(vehicle, initial, control, dt)
%OBSOLETE 2DoF flight simulation. Left in repository for future purposes of
%atmospheric ascent optimalization (2DoF is potentially faster than 3DoF).
%Vehicle is modelled as a point mass with drag. Simulation is located in a
%rotating frame of reference, so a centrifugal force appears. Uses OBSOLETE
%vehicle structure (expects to receive only one stage of an array, does not
%understand new engine struct). Atmosphere is modelled from RO data, but
%only direct drag effects are modelled (no AoA or lift).
%
%REQUIRES
%    mu         Global variable, standard gravity parameter of the body;
%               gravity constant * mass of the body (kg).
%    g0         Global variable, standard gravity acceleration (m/s).
%    R          Global variable, radius of the body (m).
%    atmpressure    Global variable, atmospheric pressure as a function of
%               altitude; array of size (n,2), altitude (kilometres above
%               sea level) in first column, pressure (atmospheres) in second.
%    atmtemperature Global variable, atmospheric temperature as a function
%               of altitude; array of size (n,2), altitude (kilometres above
%               sea level) in first column, temperature (Kelvins) in second.
%
%INPUT
%    vehicle    Single struct of the obsolete vehicle type.
%    initial    Struct of initial conditions type.
%    control    Struct defining method of controlling the current stage.
%               Supports gravity turn, pitch program or PEG (type==3).
%    dt         Simulation precision in seconds.
%
%OUTPUT
%    results    Struct containing packed results of the simulation, from
%               short summary to orbital parameters and plots.
%               NO GUARANTEE THAT THOSE WILL BE COMPATIBLE WITH THE REST OF
%               THE PROGRAM.

    %declare globals
    global mu; global g0; global R;
    global atmpressure;  global atmtemperature;
    
    %unpack vehicle data
    m = vehicle.m0;
    isp0 = vehicle.i0;
    isp1 = vehicle.i1;
    dm = vehicle.dm;
    maxT = vehicle.mt;
    engT = vehicle.et;
    area = vehicle.ra;
    dragcurve = vehicle.dc;
    
    %define control type
    if control.type == 0
        %type 0 = natural gravity turn simulation
        gtiP = control.p;   %initial pitchover angle for gravity turn
        gtiV = control.v;   %velocity at which the pitchover begins
        GT = 0; %gravity turn status flag: 0 - not begun yet; 1 - equaling to flight angle; 2 - match flight angle
    elseif control.type == 1
        %type 1 = pitch program control
        prog = control.program;
    elseif control.type == 2
        %type 2 = powered explicit guidance
        target = control.target*1000+R; %target orbit altitude
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
    end
    
    %SIMULATION SETUP
    %simulate engine ignition effects
    m = m - engT*dm;    %rocket is burning fuel while bolted to the launchpad for engT seconds before it's released
    maxT = maxT - engT; %this results in loss of propellant mass and hence reduction of maximum burn time
    N = floor(maxT/dt)+1;%simulation steps
    t = zeros(1,N);     %simulation time
    Ca = 0;             %centrifugal acceleration [m/s^2] (we use a cartesian non rotating frame of reference)
    F = zeros(1,N);     %thrust [N]
    acc = zeros(1,N);   %acceleration due to thrust [m/s^s]
    q = zeros(1,N);     %dynamic pressure q [Pa]
    D = zeros(1,N);     %drag [N]
    Ga = 0;             %current gravity acceleration [m/s^2]
    vair = 0;           %relative air velocity (subtracted for aero calculations) [m/s]
    vx = zeros(1,N);    %horizontal (tangential) velocity (orbital) [m/s]
    vy = zeros(1,N);    %vertical (radial) velocity (altitude change) [m/s]
    angleS = zeros(1,N);%velocity vector direction log [deg] (0 - straight up, 90 - due east)
    angleO = zeros(1,N);%angleS is related to surface (air) (undefined on launchpad), angleO is orbital (90 on launchpad)
    pitch = zeros(1,N); %vehicle orientation log [deg] (pitch commands) (0 - straight up, 90 - due east)
    vg = 0;             %gravity losses (integrated) [m/s]
    vd = 0;             %aerodynamic losses (integrated) [m/s]
    alt = zeros(1,N);   %altitude (integral) [m] (measured from the surface)
    rad = zeros(1,N);   %radial distance from launch point (eg. geographical longitude, for a GTO launch) [deg]
    
    %initialize simulation
    if initial.type==0     %launch from static position
        alt(1) = initial.alt;
        rad(1) = initial.lon;
        vair = (2*pi*R/24/3600)*cosd(initial.lat); %gained from Earth's rotational motion
        vx(1) = vair;
        angleS(1) = 0;
        angleO(1) = 90;
    elseif initial.type==1 %vehicle already in flight
        t(1) = initial.t;
        alt(1) = initial.alt;
        rad(1) = initial.rad;
        vx(1) = initial.vx;
        vy(1) = initial.vy;
        vair = initial.wind;
        angleS(1) = asind((vx(1)-vair) / sqrt((vx(1)-vair)^2+vy(1)^2));
        angleO(1) = asind(vx(1) / sqrt(vx(1)^2+vy(1)^2));
    end
    
    %PEG setup
    if control.type==2
        dbg = zeros(6,N);  %debug log (A, B, C, sum, sum acos, T)
        %get effective exhaust velocity...
        p = approxFromCurve(alt(1)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        ve = isp*g0;
        acc(1) = ve*dm/m;  %...to find initial acceleration
        [A, B, C, T] = poweredExplicitGuidance(...
                        0,...
                        alt(1)+R, vx(1), vy(1), target,...
                        acc(1), ve,...
                        0, 0, maxT);
        pitch(1) = acosd(A + C);
    end
    
    %MAIN LOOP
    for i=2:N
        %PITCH CONTROL SECTION
        if control.type == 0
            %natural, lock-prograde gravity turn
            if vy(i-1) >= gtiV && GT == 0
                GT = 1;
            elseif angleS(i-1) > gtiP && GT == 1
                GT = 2;
            end;
            if GT == 0
                pitch(i) = 0;
            elseif GT == 1
                pitch(i) = min(pitch(i-1)+dt, gtiP);    %hardcoded 1deg/s change (to simulate real, not instantaneous pitchover)
            else
                pitch(i) = angleS(i-1);
            end;
        elseif control.type == 1
            %pitch program control
            pitch(i) = approxFromCurve(t(i-1), prog);
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
                                lc,...
                                alt(i-1)+R, vx(i-1), vy(i-1), target,...
                                acc(i-1), isp*g0,...%use previously calculated acceleration and Isp
                                A, B, T);   %passing old T instead of T-dt IS CORRECT
                lc = 0; %think about not resetting this one if PEG skipped AB recalculation
            end;
            temp = A - B*lc + C;
            %PEG debug logs
            dbg(1,i) = A;
            dbg(2,i) = B;
            dbg(3,i) = C;
            dbg(4,i) = temp;
            dbg(5,i) = acosd( min(1, max(-1, temp)) );   %clamp to arcus cosinus domain (ideally should never be needed)
            dbg(6,i) = T-lc;
            %PEG-scheduled cutoff
            if (T-lc < dt && ENG == 1)
                ENG = 2;
                break;
            end;
            %pitch control (clamped same as debug entry 5)
            pitch(i) = acosd( min(1, max(-1, temp)) );
        end
        
        %PHYSICS SIMULATION PART
        %get isp
        p = approxFromCurve(alt(i-1)/1000, atmpressure); %get pressure
        isp = (isp1-isp0)*p+isp0;
        %calculate thrust and acceleration
        if control.type==3
            %set thrust to zero for unpowered flight
            F(i) = 0;
        else
            F(i) = isp*g0*dm;
        end;
        acc(i) = F(i)/m;
        %new velocity
        dv = acc(i) * dt;
        vx(i) = vx(i-1) + dv*sind(pitch(i-1));
        vy(i) = vy(i-1) + dv*cosd(pitch(i-1));
        %centrifugal and gravitational acceleration
        Ca = vx(i-1)^2/(alt(i-1)+R);
        Ga = mu/(alt(i-1)+R)^2;
        vy(i) = vy(i) + (Ca - Ga) * dt;
        vg = vg + Ga * dt;%integrate gravity losses
        %aerodynamic losses (simplest way: no lift or AoA effects)
            %velocity related to air (so we subtract what we gained with
            %Earth's rotation because the air around us is rotating too)
        airv = vy(i-1)^2 + ((vx(i-1))-vair)^2;
            %we retain squared value for dynamic pressure calculation, for
            %drag coefficient we do separate ad-hoc root
        cd = approxFromCurve(sqrt(airv), dragcurve);
        temp = approxFromCurve(alt(i-1)/1000, atmtemperature)+273.15;
        dens = calculateAirDensity(p*101325, temp);
        q(i) = 0.5*dens*airv;   %dynamic pressure
        D(i) = area*cd*q(i);    %aerodynamic drag force (original formula: 0.5*A*cd*dens*airv^2)
        dv = D(i)/m * dt;       %reuse an old variable, this time for a velocity decrement due to drag
        vx(i) = vx(i) - dv*sind(angleS(i-1));
        vy(i) = vy(i) - dv*cosd(angleS(i-1));
        vd = vd + dv;           %integrate aerodynamic losses
        %velocity vector angles
        angleS(i) = asind((vx(i)-vair) / sqrt((vx(i)-vair)^2+vy(i)^2)); %surface-related
        angleO(i) = asind(vx(i) / sqrt(vx(i)^2+vy(i)^2));    %orbit-related
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
    plots = struct('t', t(1:i-1),...
                   'F', F(1:i-1),...
                   'a', acc(1:i-1),...
                   'q', q(1:i-1),...
                   'D', D(1:i-1),...
                   'vx', vx(1:i-1),...
                   'vy', vy(1:i-1),...
                   'angle_ps', angleS(1:i-1),...
                   'angle_po', angleO(1:i-1),...
                   'pitch', pitch(1:i-1),...
                   'alt', alt(1:i-1),...
                   'rad', rad(1:i-1));
    %add debug data if created
    if exist('dbg', 'var')==1
        plots().DEBUG = dbg;
    end;
    results = struct('Altitude', alt(i-1),...
                     'Apoapsis', 0, 'Periapsis', 0,...
                     'Eccentricity', 0,...
                     'Velocity', sqrt(vx(i-1)^2+vy(i-1)^2),...
                     'VelocityX', vx(i-1),...
                     'VelocityY', vy(i-1),...
                     'maxQv', 0, 'maxQt', 0,...
                     'Angle_srf', angleS(i-1),...
                     'Angle_obt', angleO(i-1),...
                     'LostGravity', vg,...
                     'LostDrag', vd,...
                     'LostTotal', vg+vd,...
                     'BurnTimeLeft',maxT-t(i-1)+t(1),...
                     'Plots', plots);
    [results.Apoapsis,results.Periapsis,results.Eccentricity] = getOrbital(vx(i-1), vy(i-1), alt(i-1)+R); %returns apoapsis and periapsis
    [results.maxQt, results.maxQv] = getMaxValue(q); %returns maxQ's occurence (index in time table) and value
    results.maxQt = t(results.maxQt); %format to seconds