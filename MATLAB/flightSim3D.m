%flightSim3D.m
%Experiment on 3DoF simulation in Cartesian coordinates.
%Progress map:
%   [+] reconstruct world and variables
%   [+] basic data output for verification
%   [+] unpowered motion in gravity
%   [+] basic 3D visualization
%   [+] thrust
%   [ ] drag
%   [ ] natural gravity turn
%   [ ] pitch/yaw programming
%   [ ] ?
function [results] = flightSim3D(vehicle, time, dt)
    %declare globals
    global mu, global g0, global R;
    global atmpressure;
    
    %unpack vehicle data
    m = vehicle.m0;
    isp0 = vehicle.i0;
    isp1 = vehicle.i1;
    dm = vehicle.dm;
    maxT = vehicle.mt;%time
    engT = vehicle.et;
    area = vehicle.ra;
    dragcurve = vehicle.dc;
    
    %SIMULATION SETUP
    m = m - engT*dm;    %rocket is burning fuel while bolted to the launchpad for engT seconds before it's released
    maxT = maxT - engT; %this results in loss of propellant mass and hence reduction of maximum burn time
    N = floor(maxT/dt)+1;%simulation steps
    t = zeros(N,1);     %simulation time
    F = zeros(N,1);     %thrust magnitude [N]
    acc = zeros(N,1);   %acceleration due to thrust magnitude [m/s^2]
    %vehicle position in cartesian XYZ frame
    r = zeros(N,3);     %from Earth's center [m]
    rr = zeros(N,1);    %magnitude [m]
    %vehicle velocity
    v = zeros(N,3);     %relative to Earth's center [m/s]
    vv = zeros(N,1);    %magnitude [m/s]
    %flight angles (THIS IS NOT RIGHT FOR NOW)
    fpa_srf = zeros(1,N);   %Flight Pitch Angle (surface) - how much the vehicle points "up" in relation to moving air [deg]
                            %at launchpad this is undefined. 0 = straight up, 90 = due East
    fpa_obt = zeros(1,N);   %the same but related to orbital velocity [deg]le points "up" in relation to moving air [deg]
                            %at launchpad this equals 90 (only Earth's rotation)
    fya_srf = zeros(1,N);   %Flight Yaw Angle (surface) - how much the vehicle points "South" in relation to moving air [deg]le points "up" in relation to moving air [deg]
                            %at launchpad this is undefined (stationary related to surface/air)
                            %0 = due East, 90 = due South
    fya_obt = zeros(1,N);   %the same but related to orbital velocity [deg]
                            %at launchpad this equals 90 (craft does not move towards the equator)
                            %0 = due East, 90 = due South
    %forces
    Ga = [0 0 0];           %current gravitational acceleration vector [m/s^2]
    
    %SIMULATION INITIALIZATION (just bogus for now)
    r(1,:) = [R/sqrt(2) 0 R/sqrt(2)];
    rr(1) = norm(r(1,:));
    v(1,:) = [0 7863.3122 0];
    vv(1) = norm(v(1,:));
    ru = r(1,:)/rr(1);
    nu = cross(r(1,:),v(1,:)); nu = nu/norm(nu);
    cu = cross(nu,ru);
    un = [r(1,1) r(1,2) 0]; un = un/norm(un);
    nn = cross([r(1,1) r(1,2) 0],[v(1,1) v(1,2) 0]); nn = nn/norm(nn);
    en = cross(nn,un);
    
    %MAIN LOOP
    for i=2:N
        %PHYSICS PART
        %thrust/acceleration
        p = approxFromCurve((rr(i-1)-R)/1000, atmpressure);
        isp = (isp1-isp0)*p+isp0;
        F(i) = isp*g0*dm;
        acc(i) = F(i)/m;
        acv1 = cosd(0)*ru;         %vertical component of thrust
        acv2 = sind(0)*cosd(0)*cu; %prograde component of thrust
        acv3 = sind(0)*sind(0)*nu; %normal component of thrust
        %acv = acc(i)*(acv1+acv2+acv3);
        acv = acc(i)*cFromNavball(r(i-1,:), v(i-1,:), 20, 90);
        %angle between planar component of thrust and east +angle between coordinate systems in plane
        acosd(dot(acv2+acv3,en)/(norm(acv2+acv3)*norm(en)))+acosd(dot(cu,en));
        %gravity
        Ga = mu*r(i-1,:)/norm(r(i-1,:))^3;
        %velocity
        v(i,:) = v(i-1,:) - Ga*dt + acv*dt;
        vv(i) = norm(v(i,:));
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        rr(i) = norm(r(i,:));
        %vectors
            %local coordinates (as used by PEG)
        ru = r(i,:)/norm(r(i,:));   %radial versor -> our local "Up" (away from the Earth)
        nu = cross(r(i,:),v(i,:));
        nu = nu/norm(nu);           %normal versor -> our local "Left"
        cu = cross(nu,ru);          %circuferential versor -> our local "Forward" (towards the horizon)
            %"navball" coordinates
        un = [r(i,1) r(i,2) 0];
        un = un/norm(un);           %local "Equatorial Up"
        nn = cross([r(i,1) r(i,2) 0],[v(i,1) v(i,2) 0]);
        nn = nn/norm(nn);           %local "North"
        en = cross(nn,un);          %local "East"
            %angles
        %MASS&TIME
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
    end;
    
    %OUTPUT
    plots = struct('t', t(1:i),...
                   'v', v,...
                   'vv', vv,...
                   'r', r);
    results = struct('Altitude', (norm(r(i,:))-R)/1000,...
                     'Velocity', norm(v(i,:)),...
                     'Plots', plots);
   % figure(1); clf; plot(vv);
    figure(2); clf;
    hold on;
    plot3(r(:,1),r(:,2),r(:,3));
    [sx,sy,sz]=sphere(20);%make half a sphere
    sx=sx(11:end,:);
    sy=sy(11:end,:);
    sz=sz(11:end,:);
    plot3(R*sx,R*sy,R*sz,'g'); scatter3(0,0,0,'g');
    %local circumferential versors
    scatter3(r(i,1),r(i,2),r(i,3),'r');
    scale=2000000;
    t=zeros(2,3);
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*ru; %radial (away)
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nu; %normal (plane change)
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*cu; %circumferential (prograde)
    plot3(t(:,1),t(:,2),t(:,3),'k');
        %[dot(ru,nu) dot(ru,cu) dot(nu,cu)]
    %navball versors
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*un; %away in equatorial plane
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*nn; %north
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*en; %east
    plot3(t(:,1),t(:,2),t(:,3),'r');
        %[dot(un,nn) dot(un,en) dot(nn,en)]
    %acceleration
    t(1,:)=r(i,:); t(2,:)=t(1,:)+scale*0.03*acv;
    plot3(t(:,1),t(:,2),t(:,3),'g');
        norm(acv)
    hold off;
end

%constructs a cartesian unit vector for given navball heading
function [c] = cFromNavball(r, v, p, y)
    %pass current position under r (1x3)
    %current velocity under v (1x3)
    %pitch aim under p (scalar) - degrees away from Up
    %yaw aim under y (scalar) - degrees from East towards North
    up = [r(1) r(2) 0];
    up = up/norm(up);           %local "Equatorial Up"
    north = cross([r(1) r(2) 0],[v(1) v(2) 0]);
    north = north/norm(north);  %local "North"
    east = cross(north,up);     %local "East"
    %up, north and east components
    a = cosd(p)*up;
    b = sind(p)*sind(y)*north;
    c = sind(p)*cosd(y)*east;
    c = a + b + c;
end