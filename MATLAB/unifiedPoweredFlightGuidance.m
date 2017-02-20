function [current, guidance, debug] = unifiedPoweredFlightGuidance(vehicle, target, state, previous)
%[current, guidance, debug] = UNIFIEDPOWEREDFLIGHTGUIDANCE(vehicle,
%                                                  target, state, previous)
%Implementation of Unified Powered Flight Guidance in Standard Ascent Mode
%as described by Brand, Brown and Higgins in Space Shuttle GN&C Equation
%Document 24.
%
%REQUIRES
%    g0         Global variable, standard gravity acceleration.
%
%INPUT
%    vehicle    (Array of) struct defining vehicle performance stage by
%               stage. First element of the array should be the currently
%               flown stage.
%    target     Struct defining desired insertion conditions.
%    state      Struct defining current vehicle physical state.
%    previous   UPFG internal struct containing results of previous iteration.
%
%OUTPUT
%    current    UPFG internal struct containing results of this iteration.
%    guidance   Struct containing calculated guidance parameters.
%    debug      Struct containing raw values of produced vectors and scalars.
    
    %"BLOCK 0"
    global g0; global convergenceCriterion;
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
    SM = ones(n,1);         %thrust mode (1=const thrust, 2=const acc)
    aL = zeros(n,1);        %acceleration limit for const acceleration mode
    md = zeros(n,1);        %mass flow rate
    ve = zeros(n,1);        %effective exhaust velocity
    fT = zeros(n,1);        %thrust
    aT = zeros(n,1);        %acceleration at the beginning of stage
    tu = zeros(n,1);        %"time to burn as if the whole stage was fuel"
    tb = zeros(n,1);        %remaining burn time of each stage
    
    for i=1:n
        SM(i) = vehicle(i).MODE;
        aL(i) = vehicle(i).gLim * g0;
        [fT(i), md(i), ve(i)] = getThrust(vehicle(i).engines, 0, 0);
        ve(i) = ve(i) * g0;
        aT(i) = fT(i) / vehicle(i).m0;
        tu(i) = ve(i) / aT(i);
        tb(i) = vehicle(i).maxT;
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
    %This is done according to theory on pages 33-34 and block diagrams on
    %56-57, although with a small change: original document for the Space
    %Shuttle assumed that standard ascent will be finalized with a
    %predetermined OMS burn (Orbiter's SSMEs burning fuel from ET will only
    %take it so far, then the ET is jettisoned and vehicle coasts for a
    %predetermined amount of time (tc), after which the last burn phase
    %circularizes). Therefore, active guidance did not calculate the
    %integral Li(n). Stages 1..n-2 were assumed to burn out completely
    %(hence the logarithmic expression for them), and stage n-1 has burn
    %time set accordingly, so the total delta-v expended equals vgo.
    %In this application however, there is no such thing as a predetermined
    %final stage velocity. Therefore, stages n-1 are assumed to burn out
    %completely, and the last one is adjusted, so it burns out only as long
    %as needed.
    if SM(1)==1
        aT(1) = fT(1) / m;
    elseif SM(1)==2
        aT(1) = aL(1);
    end;
    tu(1) = ve(1) / aT(1);
    L = 0;
    Li = zeros(n,1);
    for i=1:n-1
        if SM(i)==1
            Li(i) = ve(i)*log(tu(i) / (tu(i)-tb(i)));
        elseif SM(i)==2
            Li(i) = aL(i)*tb(i);
        end;
        L = L + Li(i);
        %If we have more stages than we need to get to orbit, redo the
        %whole calculation but skip the last stage.
        if L>norm(vgo)
            [current, guidance, debug] = unifiedPoweredFlightGuidance(vehicle(1:n-1), target, state, previous);
            return;
        end
    end
    Li(n) = norm(vgo) - L;
    %Now for each stage its remaining time of burn is calculated (tbi) and
    %in the same pass accumulated into a total time-to-go of the maneuver.
    tgoi = zeros(n,1);
    for i=1:n
        if SM(i)==1
            tb(i) = tu(i)*(1-exp(-Li(i)/ve(i)));
        elseif SM(i)==2
            tb(i) = Li(i) / aL(i);
        end;
        if i==1
            tgoi(i) = tb(i);
        else
            tgoi(i) = tgoi(i-1) + tb(i);
        end
    end
    L1 = Li(1);
    tgo = tgoi(n);
    
    %BLOCK 4
    L = 0;
    J = 0; Ji = zeros(n,1);
    S = 0; Si = zeros(n,1);
    Q = 0; Qi = zeros(n,1);
    H = 0;
    P = 0; Pi = zeros(n,1);
    %Major loop of the whole block, almost exactly as in the block diagrams.
    for i=1:n
        %Variable tgoi1 represents t_go,i-1 only is determined in a safe
        %way (as to not exceed the array).
        if i==1
            tgoi1 = 0;
        else
            tgoi1 = tgoi(i-1);
        end
        
        %Constant thrust vs constant acceleration mode
        if SM(i)==1
            Ji(i) = tu(i)*Li(i) - ve(i)*tb(i);
            Si(i) = -Ji(i) + tb(i)*Li(i);
            Qi(i) = Si(i)*(tu(i)+tgoi1) - (1/2)*ve(i)*tb(i)^2;
            Pi(i) = Qi(i)*(tu(i)+tgoi1) - (1/2)*ve(i)*tb(i)^2 * ((1/3)*tb(i)+tgoi1);
        elseif SM(i)==2
            Ji(i) = 0.5*Li(i)*tb(i);
            Si(i) = Ji(i);
            Qi(i) = Si(i)*((1/3)*tb(i) + tgoi1);
            Pi(i) = (1/6)*Si(i)*(tgoi(i)^2 + 2*tgoi(i)*tgoi1 + 3*tgoi1^2);
        end
        
        %Common for both modes
        Ji(i) = Ji(i) + Li(i)*tgoi1;
        Si(i) = Si(i) + L*tb(i);
        Qi(i) = Qi(i) + J*tb(i);
        Pi(i) = Pi(i) + H*tb(i);
        
        %No coast period before the last stage.
        
        L = L + Li(i);
        J = J + Ji(i);
        S = S + Si(i);
        Q = Q + Qi(i);
        P = P + Pi(i);
        H = J*tgoi(i) - Q;
    end
    
    %BLOCK 5
    lambda = unit(vgo);
    rgrav1 = rgrav;
    if previous.tgo~=0
        rgrav = (tgo/previous.tgo)^2 * rgrav;
    end;
    rgo = rd - (r + v*tgo + rgrav); rgo1 = rgo;
    iz = unit(cross(rd,iy)); iz1 = iz;
    rgoxy = rgo - dot(iz,rgo)*iz;
    rgoz = (S - dot(lambda,rgoxy)) / dot(lambda,iz);
    rgo = rgoxy + rgoz*iz + rbias;
    lambdade = Q - S*J/L;
    lambdadot = (rgo - S*lambda) / lambdade;
    iF = unit(lambda - lambdadot*J/L);
    phi = acos(dot(iF,lambda));
    phidot = -phi*L/J;
    vthrust = (L - 0.5*L*phi^2 - J*phi*phidot - 0.5*H*phidot^2)*lambda;
    vthrust = vthrust - (L*phi + J*phidot)*unit(lambdadot);
    rthrust = (S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2)*lambda;
    rthrust = rthrust - (S*phi + Q*phidot)*unit(lambdadot);
    vbias = vgo-vthrust;
    rbias = rgo-rthrust;
    
    %BLOCK 6 - original document does not contain any implementation
    %TODO - pitch and yaw RATES
    UP = unit(r);
    EAST = unit(cross([0,0,1], UP));
    frame = [UP;[0,0,1];EAST];
    pitch = getAngleFromFrame(iF, frame, 'pitch');
    yaw = getAngleFromFrame(iF, frame, 'yaw');
    
    %BLOCK 7 - this calls the Conic State Extrapolation Routine
    rc1 = r - 0.1*rthrust - (1/30)*vthrust*tgo;
    vc1 = v + 1.2*rthrust/tgo - 0.1*vthrust;
    [rc2, vc2, cser] = CSEroutine(rc1, vc1, tgo, cser);
    %[rc2, vc2, cser] = easyCSE(rc1, vc1, tgo, cser);
    vgrav = vc2 - vc1;
    rgrav = rc2 - rc1 - vc1*tgo;
    
    %BLOCK 8
    rp = r + v*tgo + rgrav + rthrust;
    rp = rp - dot(rp,iy)*iy;
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
               'vgo2', vgo,...
               'diverge', 0);
end