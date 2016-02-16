%ascentSimulationPEG.m
%2DOF exoatmospheric ascent simulation with Powered Explicit Guidance.
%Pass initial conditions in 'init' struct. Pass vehicle parameters in
%'vehicle' struct. Pass PEG control data in 'control'. Pass simulation
%step time in 'dt'.
%Vehicle is modelled as a dragless point mass. Simulation is located in a
%rotating frame of reference, so a centrifugal force appears. Vehicle is
%assumed to only have one engine and fuel tank. Full vacuum is assumed.
%Dependencies:
%   poweredExplicitGuidance.m
%TODO: reuse ascentSimulation code (merge). Unify vehicle data structure
%throughout various simulations and get rid of globals. Adjust the control
%data structure accordingly. Enable initial conditions in ascentSimulation.
%Allow for coast periods (unpowered flight) simulation.
function [results] = ascentSimulationPEG(vehicle, init, control, dt)
    %declare globals
    global mu; global g0; global R;
    
    %unpack vehicle data
    m = vehicle.m0;
    isp = vehicle.i0;
    dm = vehicle.dm;
    maxT = vehicle.mt;
    
    %simulation setup
    lc = 0;             %time from last PEG cycle
    N = floor(maxT/dt)+1;
    t = zeros(1,N);     %time vector
    dbg = zeros(6,N);   %debug log (A, B, C, sum, sum acos, T)
    C = 0;              %centrifugal acceleration [m/s2]
    G = 0;              %gravity acceleration [m/s2]
    vg = 0;             %gravity losses vs centrifugal gains (at current iteration) [m/s]
    pitch = zeros(1,N); %guidance commands log [deg] (0 - straight up, 90 - due east)
    vx = zeros(1,N);    %horizontal (tangential) velocity (towards the orbital velocity)
    vy = zeros(1,N);    %vertical (radial) velocity (altitude change)
    alt = zeros(1,N);   %altitude (distance from Earth's centre) [m]
    rad = zeros(1,N);   %radial distance from ignition point [deg]
    ENG = 1;            %engine state flag: 0 - fuel deprived; 1 - running; 2 - cut by PEG
    cop = 1;            %cutoff point (iterator value, not time)
    
    %unpack init data
    t(1) = 0;%init.t;
    alt(1) = init.alt;
    rad(1) = init.rad;
    vx(1) = init.vx;
    vy(1) = init.vy;
    
    %unpack control data
    target = control.target*1000+R;
    ct = control.ct;
    
    %PEG setup and first run
    ve = isp*g0;       %effective exhaust velocity
    acc = ve*dm/m;     %initial acceleration
    [A, B, C, T] = poweredExplicitGuidance(...
                    0,...
                    alt(1), vx(1), vy(1), target,...
                    acc, ve,...
                    0, 0, maxT);
    
	%MAIN SIMULATION LOOP
    for i=2:N
        %start with time update
        t(i) = t(i-1) + dt;
        %check if there's still fuel
        if (t(i) > maxT && ENG > 0)
            ENG = 0;    %engine ran out of fuel
            cop = i;    %remember cutoff time
        end;

        %PEG NAVIGATION PART
        if (ENG == 1)   %if the engine is firing
            %get current vehicle acceleration
            acc = ve*dm/m;
            %acc = acc*(1+0.02*sin(2*pi/2.0*t(i)));%simulate pogo thrust oscillation, period T=2.0s, amplitude A = 2%
            %check how long ago was the last PEG cycle
            if (lc < ct)
                %if not too long ago - increment
                lc = lc + dt;
            else
                %run PEG
                [A, B, C, T] = poweredExplicitGuidance(...
                                lc,...
                                alt(i-1), vx(i-1), vy(i-1), target,...
                                acc, ve,...
                                A, B, T);   %passing old T instead of T-dt IS CORRECT
                lc = 0;
            end;
            temp = A - B*lc + C;
            %PEG debug logs
            dbg(1,i) = A;
            dbg(2,i) = B;
            dbg(3,i) = C;
            dbg(4,i) = temp;
            dbg(5,i) = acosd( min(1, max(-1, temp)) );   %clamp to arcus cosinus domain (ideally should never be needed)
            dbg(6,i) = T-lc;
        else
            acc = 0; %if engine was cut, there's no acceleration
        end;
        %PEG-scheduled cutoff
        if (T-lc < dt && ENG == 1)
            ENG = 2;
            cop = i;
        end;
        %pitch control (clamped same as debug entry 5)
        pitch(i) = acosd( min(1, max(-1, temp)) );

        %PHYSICS SIMULATION PART
        %thrust is constant, atmosphere is not present;
        %calculate gravity and centrifugal components...
        ca = vx(i-1)^2 / alt(i-1);
        ga = mu / alt(i-1)^2;
        vg = (ca-ga)*dt;
        %...and delta-v both directions
        vy(i) = vy(i-1) + vg + acc*cosd(pitch(i)) * dt;
        vx(i) = vx(i-1) + (acc*sind(pitch(i))) * dt;
        %integrate altitude and radial distance
        alt(i) = alt(i-1) + vy(i) * dt;
        rad(i) = rad(i-1) + atan2d( vx(i)*dt, alt(i) );
        %if the engine is running, update mass
        if (ENG == 1)
            m = m - dm*dt;
        end;
        %legacy: collision checking
        if (alt(i) < R)
            break;
        end;
    end
    %prepare summary
    plots = struct('t', t,...
                   'vx', vx,...
                   'vy', vy,...
                   'pitch', pitch,...
                   'altitude', alt,...
                   'radial', rad);
    airv = sqrt(vx(cop)^2+vy(cop)^2);
    results = struct('Apoapsis', 0, 'Periapsis', 0,...
                     'Eccentricity', 0,...
                     'Velocity', airv,...
                     'Altitude', (alt(cop)-R)/1000,...
                     'VelocityX', vx(cop),...
                     'VelocityY', vy(cop),...
                     'BurnTimeLeft', maxT-t(cop),...
                     'Plots', plots, 'DEBUG', dbg,...
                     'eng',ENG);
    [results.Apoapsis,results.Periapsis,results.Eccentricity] = getOrbital(vx(cop), vy(cop), alt(cop)); %returns apoapsis and periapsis
end