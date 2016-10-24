%unifiedPoweredFlightGuidance.m
%Implementation of Unified Powered Flight Guidance in Standard Ascent Mode
%as described by Brand, Brown and Higgins in Space Shuttle GN&C Equation Document 24.
function [current, guidance, debug] = unifiedPoweredFlightGuidance(vehicle, target, state, previous)
    %INPUT
    %vehicle    array of struct, defines vehicle performance data stage-by-stage
    %           see PM2 file for definition
    %           UPFG expects a list of stages from currently flown up (so
    %           for a 4-stage-total vehicle, currently flying the 3rd
    %           stage, only pass stages 3:4)
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
    % .tb       value in seconds, elapsed time of this burn phase
    % .time     value in seconds, previous iteration time
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
    tp      = previous.time;
    vprev   = previous.v;
    vgo     = previous.vgo;
    
    %BLOCK 1
    n  = length(vehicle);   %total number of stages
    md = zeros(n,1);        %mass flow rate
    ve = zeros(n,1);        %effective exhaust velocity
    fT = zeros(n,1);        %thrust
    aT = zeros(n,1);        %acceleration at the beginning of phase
    tu = zeros(n,1);        %"time to burn as if the whole stage was fuel"
    tb = zeros(n,1);        %remaining burn time of each stage
                            %to calculate t_go,i from that, just sum all
                            %the way from 1 to i
    for i=1:n
        md(i) = vehicle(i).dm;
        ve(i) = vehicle(i).i0 * g0;
        fT(i) = md(i) * ve(i);
        aT(i) = fT(i) / vehicle(i).m0;
        tu(i) = ve(i) / aT(i);
        tb(i) = vehicle(i).mt;
    end
    
    %BLOCK 2
    %We need to store dt in order to keep track on maneuver time.
    dt = t-tp;
    %In the paper, this block assumes the only known thing about vehicle's
    %state change since the last iteration is vector of velocity change
    %(delta-v_sensed) and time interval (delta t). In this implementation
    %however we assume the state is perfectly known and hence the call to
    %Powered Flight Navigation Routine is not necessary.
    %However, we still decrement vgo here!
    dvsensed = v - vprev;
    vgo = vgo - dvsensed; vgo1 = vgo;
    %Calculation of 'remaining time to burn in stage k' (which here means
    %current stage) is done differently than in the paper. There, t_b,k is
    %decremented by dt every iteration; here this variable is not
    %persistent, but re-read from vehicle data at each iteration, and
    %instead we remember the current burn time tb, so we simply subtract
    %this time from original time-to-burn, obtaining the remaining time.
    tb(1) = tb(1) - previous.tb;
    
    %BLOCK 3
    %Current vehicle parameters have already been obtained in block 1, the
    %only thing different is a_T,k which should be calculated from current
    %mass instead of initial, and subsequently tu,k.
    %This is done according to theory on pages 33-34 and block diagrams on 56-57.
    aT(1) = fT(1) / m;
    tu(1) = ve(1) / aT(1);
    L = 0;
    Li = zeros(n,1);
    for i=1:n-2
        Li(i) = ve(i)*log(tu(i) / (tu(i)-tb(i)));
        L = L + Li(i);
    end
    if n>1
        Li(n-1) = norm(vgo) - L - Li(n);
    else
        Li(n) = norm(vgo);
    end
    %The above made sense for the Space Shuttle, whose last stage was post
    %ET jettison orbit circularisation on OMS, which apparently had a
    %predetermined amount of delta-v. So the Li array was not initialized
    %fully with zeros, but the n-th element was this predetermined d-v. I
    %left the subtraction (line right after "if n>1") because it does no
    %harm here.
    tgoi = zeros(n,1);
    for i=1:n
        tb(i) = tu(i)*(1-exp(-Li(i)/ve(i)));
        if i==1
            tgoi(i) = tb(i);
        else
            tgoi(i) = tgoi(i-1) + tb(i);
        end
    end
    %Li'
    %tgoi'
    L1 = Li(1);
    tgo = tgoi(1);
    
    %BLOCK 4
    L = 0;
    J = 0; Ji = zeros(n,1);
    S = 0; Si = zeros(n,1);
    Q = 0; Qi = zeros(n,1);
    H = 0;
    P = 0; Pi = zeros(n,1);
    %Major loop of the whole block
    for i=1:n
        Ji(i) = tu(i)*Li(i) - ve(i)*tb(i);
        Si(i) = -Ji(i) + tb(i)*Li(i);
        if i==1
            tgoi1 = 0;
        else
            tgoi1 = tgoi(i-1);
        end
        Qi(i) = Si(i)*(tu(i)+tgoi1) - (1/2)*ve(i)*tb(i)^2;
        Pi(i) = Qi(i)*(tu(i)+tgoi1) - (1/2)*ve(i)*tb(i)^2 * ((1/3)*tb(i)+tgoi1);
        
        Ji(i) = Ji(i) + Li(i)*tgoi1;
        Si(i) = Si(i) + L*tb(i);
        Qi(i) = Qi(i) + J*tb(i);
        Pi(i) = Pi(i) + H*tb(i);
        
        %no pre-last stage coast period
        L = L + Li(i);
        J = J + Ji(i);
        S = S + Si(i);
        Q = Q + Qi(i);
        P = P + Pi(i);
        H = J*tgoi(i) - Q;
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
        lambdade = Q - S*J/L;
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
    %TODO - pitch and yaw RATES
    pitch = acosd(dot(iF,unit(r)));         %angle between thrust vector and local UP
    iF_up = dot(iF,unit(r))*unit(r);        %thrust component in UP direction
    iF_plane = unit(iF - iF_up);            %sphere-tangential plane component of thrust
    EAST = cross([0,0,1],unit(r));          %local EAST direction
    NORTH = cross(unit(r),EAST);            %local NORTH
    yaw = 90-acosd(dot(iF_plane,NORTH));    %yaw is between in-plane thrust and NORTH
                                            %but zeroes at full EAST direction
    
    %BLOCK 7 - this calls the Conic State Extrapolation Routine
    rc1 = r - 0.1*rthrust - (1/30)*vthrust*tgo;
    vc1 = v + 1.2*rthrust/tgo - 0.1*vthrust;
    %[rc2, vc2, cser] = CSEroutine(rc1, vc1, tgo, cser);
    [rc2, vc2, cser] = easyCSE(rc1, vc1, tgo, cser);
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
    current.tb      = current.tb + dt;
    current.time    = t;
    current.tgo     = tgo;
    current.v       = v;
    current.vgo     = vgo;
    
    guidance = struct('pitch', pitch, 'yaw', yaw, 'pitchdot', 0, 'yawdot', 0, 'tgo', tgo);
    
    debug = struct('time', t,...
               'r', r,...
               'v', v,...
               'm', m,...
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
               'lambdade', lambdade,...
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