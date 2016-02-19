%getOrbitalElements.m
%Given the craft's position and velocity vectors (1x3) calculates its
%Keplerian orbital elements and returns them in order:
%   apoapsis
%   periapsis (bonus)
%   semi-major axis
%   eccentricity
%   inclination
%   longitude of the ascending node
%   argument of periapsis
%   true anomaly
%https://en.wikibooks.org/wiki/Astrodynamics/Classical_Orbit_Elements
%http://space.stackexchange.com/a/1919
function [ap, pe, sma, ecc, inc, lan, aop, v] = getOrbitalElements(r, v)
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
    %argument of periapsis
    aop = acosd(dot(n, e)/(norm(n)*norm(e)));
    %true anomaly
    v = acosd(dot(e, r)/(norm(e)*norm(r)));
    %Ap, Pe
    ap = (sma*(1+ecc)-R)/1000;
    pe = (sma*(1-ecc)-R)/1000;
end