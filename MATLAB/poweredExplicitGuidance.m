%poweredExplicitGuidance.m
%Implementation of Powered Explicit Guidance major loop.
%http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19660006073.pdf
%http://www.orbiterwiki.org/wiki/Powered_Explicit_Guidance
function [A, B, C, T] = poweredExplicitGuidance(cycle,   alt, vt, vr, tgt,   acc, ve,   oldA, oldB, oldT)
    %cycle  - length of the major computer cycle (time between PEG calculations) [s]
    %alt    - current altitude as distance from body center [m]
    %vt     - tangential velocity (horizontal - orbital) [m/s]
    %vr     - radial velocity (veritcal - away from the Earth) [m/s]
    %tgt    - target altitude (measured as current) [m]
    %acc    - current vehicle acceleration [m/s^2]
    %ve     - effective exhaust velocity (isp*g0) [m/s]
    %basing on current state vector, vehicle params and given ABT estimates
    %new A and B, and calculates new T from them
    %also outputs a C component for the purposes of guidance
    %passing A=B=0 causes estimation of those directly from the given oldT
    global mu;
    tau = ve / acc;
    if oldA==0 && oldB==0
        %calculate A and B (PEG.pdf, page 22)
        b0 = -ve*log(1-oldT/tau);
        b1 = b0*tau - ve*oldT;
        c0 = b0*oldT - b1;
        c1 = c0*tau - ve*oldT^2/2;
        matA = [b0 b1;
                c0 c1];
        matB = [-vr;
                tgt - alt - vr*oldT];
        matX = linsolve(matA, matB);
        oldA = matX(1);
        oldB = matX(2);
    end;
        
    %current and target angular momentum, change in momentum
    angM = norm(cross([alt 0 0], [vr; vt; 0]));
    tgtV = sqrt(mu/tgt);
    tgtM = norm(cross([tgt 0 0], [0; tgtV; 0]));
    dMom = tgtM - angM;
    
    %based on old ABT (or freshly estimated values) calculate steering constant series f_r (eq 22)
    C = (mu/tgt^2 - tgtV^2/tgt) / (acc / (1-oldT/tau));  %target values
    f_r_T = oldA + oldB*oldT + C;
    C = (mu/alt^2 - vt^2/alt) / acc;    %current values
    f_r = oldA + C;
    f_r_dot = (f_r_T - f_r) / oldT;
    
    %based on that series calculate steering constant series f_theta (eq 25)
    %f_h terms omitted because we're not doing yaw steering (yet?)
    f_theta = 1 - f_r^2/2;
    f_theta_dot = -f_r*f_r_dot;
    f_theta_2dot = -f_r_dot^2/2;
    
    %based on that series and old-but-updated T (old - delta T) calculate delta-v (eq 36)
    avgR = (alt + tgt)/2;
    deltav = dMom/avgR + ve*(oldT-cycle)*(f_theta_dot+f_theta_2dot*tau) + f_theta_2dot*ve*(oldT-cycle)^2/2;
    deltav = deltav / (f_theta + f_theta_dot*tau + f_theta_2dot*tau^2);
    
    %based on that delta-v calculate the new T (eq 37b)
    T = tau*(1-exp(-deltav/ve));
    
    %solve matrix for new steering constants AB
    %however, don't do it if T is too small
    if (T>=7.5)
        b0 = -ve*log(1-T/tau);
        b1 = b0*tau - ve*T;
        c0 = b0*T - b1;
        c1 = c0*tau - ve*T^2/2;
        matA = [b0 b1;
                c0 c1];
        matB = [-vr;
                tgt - alt - vr*T];
        matX = linsolve(matA, matB);
        A = matX(1);
        B = matX(2);
    else
        %in such case, just live with the old values
        A = oldA;
        B = oldB;
    end;
end