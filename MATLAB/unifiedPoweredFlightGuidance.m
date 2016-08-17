%unifiedPoweredFlightGuidance.m
%Implementation of Unified Powered Flight Guidance in Standard Ascent Mode
%as described by Brand, Brown and Higgins in Space Shuttle GN&C Equation Document 24.
function [current, guidance, debug] = unifiedPoweredFlightGuidance(vehicle, target, state, previous)
    %INPUT
    %vehicle    struct, defines vehicle performance data
    % .thrust   value in Newtons, assumed constant throughout the burn
    % .isp      value in seconds, assumed constant throughout the burn
    % .mass     value in kilograms, initial mass of the vehicle on ignition
    %target     struct, defines desired insertion parameters
    % .radius   value in meters (from origin, not surface)
    % .velocity value in meters per second, magnitude of vector
    % .angle    value in degrees, flight path angle
    % .normal   unit 3-vector normal to target orbital plane
    %state      struct, defines current vehicle physical state
    % .time     value in seconds, current time
    % .mass     value in kilograms, current vehicle mass
    % .radius   3-vector in meters, current vehicle position (cartesian)
    % .velocity 3-vector in meters per second, current vehicle velocity
    %previous   struct, contains results of previous iteration
    % .cser     struct, contains state of CSE routine (see file for details)
    % .rbias    3-vector in meters, calculated correction for effects of
    %           rotating thrust vector
    % .rd       3-vector in meters, calculated desired cutoff position
    % .rgrav    3-vector in meters, calculated second integral of effects
    %           of central gravity field
    % .tgo      value in seconds, calculated time-to-go
    % .v        3-vector in meters per second, stored velocity state from
    %           previous iteration
    % .vgo      3-vector in meters per second, calculated velocity-to-go
    %OUTPUT
    %current    struct, contains results of this iteration for use with the
    %           next one; fields are exactly the same as in 'previous'
    %guidance   struct, contains extracted guidance data
    % .pitch    value in degrees, calculated pitch angle
    % .pitchdot value in degrees per second, calculated pitch change rate
    % .yaw      value in degrees, calculated yaw angle
    % .yawdot   value in degrees per second, calculated yaw change rate
    % .tgo      value in seconds, time to cutoff
    %debug      struct, contains raw values of produced vectors and scalars
    %           see end of this function for details
    
    %"BLOCK 0"
    global g0;
    gamma	= target.angle;
    iy      = target.normal;
    rdval   = target.radius;
    vdval   = target.velocity;
    t       = state.time;
    m       = state.mass;
    r       = state.radius;
    v       = state.velocity;
    cser    = previous.cser;
    rbias   = previous.rbias;
    rd      = previous.rd;
    rgrav   = previous.rgrav;
    vprev   = previous.v;
    vgo     = previous.vgo;
    
    %BLOCK 1 - needs to be reworked for multistage later
    fT = vehicle.thrust;
    ve = vehicle.isp*g0;
    aT = fT/m;
    tu = ve/aT;
    
    %BLOCK 2 - tgo handling must be added but only for multistage
    %In the paper, this block assumes the only known thing about vehicle's
    %state change since the last iteration is vector of velocity change
    %(delta-v_sensed) and time interval (delta t). In this implementation
    %however we assume the state is perfectly known and hence the call to
    %Powered Flight Navigation Routine is not necessary.
    %However, we still decrement vgo here!
    dvsensed = v - vprev;
    vgo = vgo - dvsensed; vgo1 = vgo;
    
    %BLOCK 3 - needs to be reworked for multistage later
    L1 = norm(vgo);
    tgo = tu*(1-exp(-L1/ve));
    
    %BLOCK 4 - needs to be reworked for multistage later
    if 1
        %EXPERIMENT - LET'S FORGET SS GN&C EqD 24 FOR A WHILE
        L = L1;
        J = tu*L - ve*tgo;
        S = -J + tgo*L;
        Q = S*tu - (1/2)*ve*(tgo^2);
        P = Q*tu - (1/6)*ve*(tgo^3);
        H = J*tgo - Q;
    else
        %AND TRY TO GET BACK TO F. TEREN'S FORMULATION
        L = L1; %this is fixed; L=b0
        J = L*tu - ve*tgo;  %J=b1, stays as it was
        H = J*tu - 0.5*ve*tgo^2;    %H=b2, this is different
        S = L*tgo - J;      %S=c0, stays as it was
        Q = S*tu - 0.5*ve*tgo^2;    %Q=c1, turns out it's a minus? lol
        P = Q*tu - (1/6)*ve*tgo^3;  %P=c2 - it's the same!
    end
    
    %BLOCK 5
    lambda = unit(vgo);
    if previous.tgo~=0
        rgrav = (tgo/previous.tgo)^2 * rgrav; rgrav1 = rgrav;
    end;
    rgo = rd - (r + v*tgo + rgrav); rgo1 = rgo;
    iz = unit(cross(rd,iy)); iz1 = iz;
    rgoxy = rgo - dot(iz,rgo)*iz;
    rgoz = (S - dot(lambda,rgoxy)) / dot(lambda,iz);
    rgo = rgoxy + rgoz*iz + rbias;
    lambdadot = (rgo - S*lambda) / (Q - S*J/L);
    iF = unit(lambda - lambdadot*J/L);
    phi = acos(dot(iF,lambda));
    phidot = -phi*L/J;
    vthrust = (L - 0.5*L*phi^2 - J*phi*phidot - 0.5*H*phidot^2)*lambda;
    vthrust = vthrust - (L*phi + J*phidot)*unit(lambdadot);
    rthrust = (S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2)*lambda;
    rthrust = rthrust - (S*phi + Q*phidot)*unit(lambdadot);
    vbias = vgo-vthrust;
    rbias = rgo-rthrust;
    
    %BLOCK 6
    %TODO - pitch and yaw angles from iF vector (and change rates?)
    pitch = acosd(dot(iF,unit(r)));         %angle between thrust vector and local UP
    iF_up = dot(iF,unit(r))*unit(r);        %thrust component in UP direction
    iF_plane = iF - iF_up;                  %sphere-tangential plane component of thrust
    EAST = cross([0,0,1],unit(r));          %local EAST direction
    yaw = acosd(dot(unit(iF_plane),EAST));  %angle between thrust vector and local EAST
    yaw = real(yaw);    %TODO: investigate why do we even need this
    
    %BLOCK 7 - this calls the Conic State Extrapolation Routine
    rc1 = r - 0.1*rthrust - (1/30)*vthrust*tgo;
    vc1 = v + 1.2*rthrust/tgo - 0.1*vthrust;
    [rc2, vc2, cser] = CSEroutine(rc1, vc1, tgo, cser);
    vgrav = vc2 - vc1;
    rgrav = rc2 - rc1 - vc1*tgo;
    
    %BLOCK 8
    rp = r + v*tgo + rgrav + rthrust;
    rd = rdval*unit(rp);
    ix = unit(rd);
    iz = cross(ix,iy);
    vd = vdval*([ix;iy;iz]'*[sind(gamma);0;cosd(gamma)])';
    vgop = vd - v - vgrav + vbias;
    dvgo = 0.0*(vgop-vgo); %big values (0.8+) cause bananas; standard ascent uses 0 (?)
    vgo = vgop + dvgo;
    
    %RETURN
    current = previous;
    current.cser    = cser;
    current.rbias   = rbias;
    current.rd      = rd;
    current.rgrav   = rgrav;
    current.tgo     = tgo;
    current.v       = v;
    current.vgo     = vgo;
    
    guidance = struct('pitch', pitch, 'yaw', yaw, 'pitchdot', 0, 'yawdot', 0, 'tgo', tgo);
    
    debug = struct('time', t,...
               'dvsensed', dvsensed,...
               'vgo1', vgo1,...
               'L1', L1,...
               'tgo', tgo,...
               'L', L,...
               'J', J,...
               'S', S,...
               'Q', Q,...
               'P', P,...
               'H', H,...
               'lambda', lambda,...
               'rgrav1', rgrav1,...
               'rgo1', rgo1,...
               'iz1', iz1,...
               'rgoxy', rgoxy,...
               'rgoz', rgoz,...
               'rgo2', rgo,...
               'lambdadot', lambdadot,...
               'iF', iF,...
               'phi', phi,...
               'phidot', phidot,...
               'vthrust', vthrust,...
               'rthrust', rthrust,...
               'vbias', vbias,...
               'rbias', rbias,...
               'pitch', pitch,...
               'iF_up', iF_up,...
               'iF_plane', iF_plane,...
               'EAST', EAST,...
               'yaw', yaw,...
               'rc1', rc1,...
               'vc1', vc1,...
               'rc2', rc2,...
               'vc2', vc2,...
               'cser', cser,...
               'vgrav', vgrav,...
               'rgrav2', rgrav,...
               'rp', rp,...
               'rd', rd,...
               'ix', ix,...
               'iz2', iz,...
               'vd', vd,...
               'vgop', vgop,...
               'dvgo', dvgo,...
               'vgo2', vgo);
end

function [v] = unit(vector)
    if norm(vector)==0
        v = vector;
    else
        v = vector/norm(vector);
    end;
end