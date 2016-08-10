%unifiedPoweredFlightGuidance.m
%Implementation of Unified Powered Flight Guidance in Standard Ascent Mode
%as described by Brand, Brown and Higgins in Space Shuttle GN&C Equation Document 24.
function [current, guidance] = unifiedPoweredFlightGuidance(vehicle, target, state, previous)
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
    % .time     value in seconds, current mission elapsed time
    % .mass     value in kilograms, current vehicle mass
    % .radius   3-vector in meters, current vehicle position (cartesian)
    % .velocity 3-vector in meters per second, current vehicle velocity
    %previous   struct, contains results of previous iteration
    % .time     value in seconds, time of previous iteration
    % .tgo      value in seconds, calculated time-to-go
    % .rbias    3-vector in meters, calculated correction for effects of
    %           rotating thrust vector
    % .rd       3-vector in meters, calculated desired cutoff position
    % .rgrav    3-vector in meters, calculated second integral of effects
    %           of central gravity field
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
    
    %"BLOCK 0"
    global mu;
    global g0;
    gamma	= target.angle;
    iy      = target.normal;
    rdval   = target.radius;
    vdval   = target.velocity;
    r       = state.radius;
    v       = state.velocity;
    rbias   = previous.rbias;
    rd      = previous.rd;
    rgrav   = previous.rgrav;
    vgo     = previous.vgo;
    
    %BLOCK 1 - needs to be reworked for multistage later
    fT = vehicle.thrust;
    ve = vehicle.isp*g0;
    dm = fT/ve;
    m = state.mass;
    aT = fT/m;
    tu = ve/aT;
    
    %BLOCK 2 - unneeded since KSP makes all the data readily available
    %the only thing worth implementing (maybe) is the vgo update, so TODO
    
    %BLOCK 3 - needs to be reworked for multistage later
    L1 = norm(vgo);
    tgo = tu*(1-exp(-L1/ve));
    
    %BLOCK 4 - needs to be reworked for multistage later
    L = ve*log(tu/(tu-tgo));
    J = tu*L - ve*tgo;
    S = -J + tgo*L;
    Q = S*tu + 1/2*ve*tgo^2;
    P = Q*tu - 1/6*ve*tgo^3;
    H = J*tgo - Q;
    S = S + L*tgo;
    Q = Q + J*tgo;
    P = P + H*tgo;
    
    %BLOCK 5
    lambda = vgo/norm(vgo);
    %rgrav = (tgo/previous.tgo)^2 * rgrav;
    rgo = rd - (r + v*tgo + rgrav);
    iz = cross(rd,iy);
    iz = iz/norm(iz);
    rgoxy = rgo - dot(iz,rgo)*iz;
    rgoz = (S - dot(lambda,rgoxy)) / dot(lambda,iz);
    rgo = rgoxy + rgoz*iz + rbias;
    lambdadot = (rgo - S*lambda) / (Q - S*J/L);
    iF = lambda - lambdadot*J/L;
    iF = iF/norm(iF);
    phi = acos(dot(iF,lambda));
    phidot = -phi*L/J;
    vthrust = (L - 0.5*L*phi^2 - J*phi*phidot - 0.5*H*phidot^2)*lambda;
    vthrust = vthrust - (L*phi + J*phidot)*lambdadot/norm(lambdadot);
    rthrust = (S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2)*lambda;
    rthrust = rthrust - (S*phi + Q*phidot)*lambdadot/norm(lambdadot);
    vbias = vgo-vthrust;
    rbias = rgo-rthrust;
    
    %BLOCK 6
    %calculates pitch and yaw angles from iF vector
    dir_pseudo = [0,0,1];
    dir_up = r/norm(r);
    dir_east = cross(dir_pseudo,dir_up);
    dir_north = cross(dir_up,dir_east);
    pitch = acosd(dot(iF,dir_up));          %angle between two unit vectors
    iF_plane = iF - dot(iF,dir_up)*dir_up;  %thrust component in tangent plane
    yaw = 90-acosd(dot(iF_plane,dir_north));
    %TODO: turning rate vector into angle rates
    
    %BLOCK 7 - this needs the Conic State Extrapolation Routine
    %we'll use a naive trajectory integration for now
    rprop = zeros(ceil(tgo),3);
    vprop = zeros(ceil(tgo),3);
    rprop(1,:) = r - 0.1*rthrust - 1/30*vthrust*tgo;
    vprop(1,:) = v + 1.2*rthrust/tgo - 0.1*vthrust;
    for j=2:ceil(tgo)
        vprop(j,:) = vprop(j-1,:) - mu*rprop(j-1,:)/norm(rprop(j-1,:))^3;
        rprop(j,:) = rprop(j-1,:) + vprop(j,:);
    end
    vgrav = vprop(ceil(tgo),:) - vprop(1,:);
    rgrav = rprop(ceil(tgo),:) - rprop(1,:) - vprop(1,:)*tgo;
    
    %BLOCK 8
    rp = r + v*tgo + rgrav + rthrust;
    rd = rdval*rp/norm(rp);
    ix = rd/norm(rd);
    iz = cross(ix,iy);
    vd = (vdval*[ix;iy;iz]*[sind(gamma);0;cosd(gamma)])';
    vgo = vd - v - vgrav + vbias;
    
    %RETURN
    current = previous;
    current.time    = state.time;
    current.tgo     = tgo;
    current.rbias   = rbias;
    current.rd      = rd;
    current.rgrav   = rgrav;
    current.vgo     = vgo;
    guidance = struct('pitch', pitch, 'yaw', yaw, 'pitchdot', 0, 'yawdot', 0, 'tgo', tgo);