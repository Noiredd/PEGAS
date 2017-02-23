function [Ap, Pe, Ecc] = getOrbital(vx, vy, alt)
%[Ap, Pe, Ecc] = GETORBITAL(vx, vy, alt)
%Calculates orbital elements of a 2D orbit. Currently unused feature.
%
%REQUIRES
%    mu         Global variable, standard gravity parameter of the body;
%               gravity constant * mass of the body (kg).
%    R          Global variable, radius of the body (m).
%
%INPUT
%    vx         Horizontal velocity (m/s).
%    vy         Vertical velocity (m/s).
%    alt        Altitude from Earth's center (m).
%
%OUTPUT
%    Ap         Apoapsis from the body's surface (km).
%    Pe         Periapsis from the body's surface (km).
%    Ecc        Eccentricity

    global mu; global R;
    %at any given point on the ellipse: v = sqrt( mu*( 2/r-1/a ) )
    v = sqrt(vx^2 + vy^2);
    %a - semimajor axis: 1/a = 2/r - v^2/mu
    sma = 1/( 2/alt - v^2/mu );
    %specific orbital energy
    soe = -mu/(2*sma);
    %from orbital eccentricity equations: e = sqrt( 1 + 2*soe*h^2/mu^2 )
    %h - specific relative angular momentum: h = r*v
    srh = norm(cross([alt 0 0], [vy; vx; 0]));
    Ecc = sqrt(1 + 2*soe*srh^2 / mu^2);
    %apsi distances are given: r_a/p = a*( 1 +/- e )
    Ap = sma*(1+Ecc);   %raw value
    Ap = (Ap-R)/1000;   %return in [km]
    Pe = sma*(1-Ecc);
    Pe = (Pe-R)/1000;
end