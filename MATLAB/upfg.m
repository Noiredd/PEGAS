%upfg.m
%Implementation of Unified Powered Flight Guidance from Space Shuttle GN&C
%Equation Document 24 by Brand, Brown and Higgins.
%For now (just check how this converges) all code contained within this
%file, along with initial conditions and vehicle setup.

%INITIAL CONDITIONS
t  = 0;             %time now
m  = 7442;          %kg mass at t=0
r  = [R+100000,0,0];%position
v  = [2000,2500,0]; %velocity

%DESIRED FINAL CONDITIONS
rdval = R+250000;   %altitude
iy = [0,0,1];       %plane normal
vdval = 7850;       %velocity magnitude
gamma = 0;          %flight path angle (0 implies no vertical velocity)

%INITIALIZATION GUESSES
rd = cross(iy,r);
ix = rd/rdval;
iz = cross(ix,iy);
vd = (vdval*[ix;iy;iz]*[sind(gamma);0;cosd(gamma)])';
vgo = vd - v;

%BLOCK 1
fT = 55400;         %Newtons of thrust
dm = fT/(340*g0);   %mass flow rate
ve = 340*g0;        %effective exhaust velocity
aT = fT/m;          %acceleration at t=0
tu = m/dm;          %time-to-burn if the vehicle was all propellant
rgrav = mu/2 * r/norm(r)^3;
rbias = [0,0,0];

%BLOCK 2
%skipped, data readily available

dt = 0.1;
for i=1:5
    m = m - dm*dt;
    %BLOCK 3
    aT = fT/m;
    L1 = norm(vgo);
    tb1 = tu*(1-exp(-L1/ve));
    tgo = tb1;
    
    %BLOCK 4
    %L1 = -ve*log((tu-tb1)/tu); %this is how page 36 does it
    L = L1;
    J = tu*L1 - ve*tb1;
    S = -J + tb1*L1;
    Q = S*tu - 0.5*ve*tb1^2;
    P = Q*tu - 1/6*ve*tb1^3;
    H = J*tgo - Q;
    
    %BLOCK 5
    lambda = vgo/norm(vgo);
    %rgrav update here
    rgo = rd - (r + v*tgo + rgrav)%;
    iz = cross(rd,iy);
    iz = iz/norm(iz);
    rgoxy = rgo - dot(iz,rgo)*iz;
    rgoz = (S - dot(lambda,rgoxy)) / dot(lambda,iz);
    rgo = rgoxy + rgoz*iz + rbias;
    lambdadot = (rgo - S*lambda) / (Q - S*J/L);
    iF = lambda - lambdadot*J/L;
    iF = iF / norm(iF)%;
    phi = acosd(dot(iF,lambda));
    phidot = -phi*L/J;
    vthrust = (L - 0.5*L*phi^2 * J*phi*phidot - 0.5*H*phidot^2)*lambda;
    vthrust = vthrust - (L*phi + J*phidot)*lambdadot/norm(lambdadot)%;
    rthrust = (S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2)*lambda;
    rthtrst = rthrust - (S*phi + Q*phidot)*lambdadot/norm(lambdadot);
    vbias = vgo-vthrust%;
    rbias = rgo-rthrust;
    
    %BLOCK 7
    %we NEED to simulate coast flight here to get vgrav/rgrav
    vgrav = [0,0,0];
    
    %BLOCK 8
    rp = r + v*tgo + rgrav + rthrust;
    rd = rdval*rp/norm(rp);
    ix = rd/norm(rd);
    iz = cross(ix,iy);
    vd = (vdval*[ix;iy;iz]*[sind(gamma);0;cosd(gamma)])';
    vgo = vd - v - vgrav% + vbias%;
end