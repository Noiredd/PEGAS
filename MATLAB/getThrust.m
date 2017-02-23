function [F, dm, isp] = getThrust(engines, pressure, time)
%[F, dm, Isp] = GETTHRUST(engines, pressure, time)
%Calculates thrust and closely related parameters for the given set of
%engines at given pressure. Supports engines with constant thrust (eg.
%liquid) and those with thrust profile (eg. solid rocket motors).
%
%REQUIRES
%    g0         Global variable, standard gravity acceleration (m/s).
%
%INPUT
%    engines    Array of struct of type engine.
%    pressure   Atmospheric pressure (Pascals).
%    time       Time since ignition of the engines (seconds).
%
%OUTPUT
%    F          Combined thrust (Newtons).
%    dm         Combined mass flow rate (kg/s).
%    Isp        Combined specific impulse (seconds).

    global g0;
    n = length(engines);
    p = pressure;
    t = time;
    F = 0;
    dm = 0;
    for i=1:n
        isp1 = engines(i).isp1;
        isp0 = engines(i).isp0;
        isp = (isp1-isp0)*p+isp0;
        if engines(i).mode==1
            dm_ = engines(i).flow;
        elseif engines(i).mode==2
            dm_ = approxFromCurve(t, engines(i).data) * engines(i).flow;
        end
        dm = dm + dm_;
        F = F + isp*dm_*g0;
    end
    isp = F/(dm*g0);
end