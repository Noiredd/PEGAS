function [ap, pe, sma, ecc, inc, lan, aop, tan] = getOrbitalElements(r, v)
%[ap, pe, sma, ecc, inc, lan, aop, tan] = GETORBITALELEMENTS(r, v)
%Calculates complete set of Keplerian elements of a 3D orbit given by
%position and velocity vectors. Equations taken from:
%https://en.wikibooks.org/wiki/Astrodynamics/Classical_Orbit_Elements
%http://space.stackexchange.com/a/1919
%
%REQUIRES
%    mu         Global variable, standard gravity parameter of the body;
%               gravity constant * mass of the body (kg).
%    R          Global variable, radius of the body (m).
%
%INPUT
%    r          Position XYZ vector relative to the center of reference
%               frame (m).
%    v          Velocity XYZ vector (m/s).
%
%OUTPUT
%    ap         Apoapsis from the body's surface (km).
%    pe         Periapsis from the body's surface (km).
%    sma        Semi-major axis (m).
%    ecc        Eccentricity
%    inc        Inclination (deg).
%    lan        Longitude of ascending node (deg).
%    aop        Argument of periapsis (deg).
%    tan        True anomaly (deg).

    global mu; global R;
    %angular momentum
    h = cross(r, v);
    %ascending node
    n = cross([0 0 1], h);
    %specific mechanical energy
    E = norm(v)^2/2-mu/norm(r);
    %semi-major axis
    sma = -0.5*mu/E;
    %eccentricity vector
    e = (norm(v)^2/mu - 1/norm(r))*r - dot(r, v)/mu*v;
    ecc = norm(e);
    %inclination
    inc = acosd(dot([0 0 1], h)/norm(h));
    %longitude of the ascending node
    lan = acosd(dot([1 0 0], n)/norm(n));
    if n(2)<0
        lan = 360-lan;
    end;
    %argument of periapsis
    aop = acosd(dot(n, e)/(norm(n)*norm(e)));
    if e(3)<0
        aop = 360-aop;
    end;
    %true anomaly
    tan = acosd(dot(e, r)/(norm(e)*norm(r)));
    if dot(r,v)<0
        tan = 360-tan;
    end;
    %Ap, Pe
    ap = (sma*(1+ecc)-R)/1000;
    pe = (sma*(1-ecc)-R)/1000;
end