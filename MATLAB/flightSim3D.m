%flightSim3D.m
%Experiment on 3DoF simulation in Cartesian coordinates.
%Progress map:
%   [+] reconstruct world and variables
%   [+] basic data output for verification
%   [+] unpowered motion in gravity
%   [+] basic 3D visualization
%   [ ] thrust
%   [ ] drag
%   [ ] natural gravity turn
%   [ ] pitch/yaw programming
%   [ ] ?
function [results] = flightSim3D(vehicle, time, dt)
    %declare globals
    global mu, global g0, global R;
    
    %unpack vehicle data
    m = vehicle.m0;
    isp0 = vehicle.i0;
    isp1 = vehicle.i1;
    dm = vehicle.dm;
    maxT = time;%vehicle.mt;
    engT = vehicle.et;
    area = vehicle.ra;
    dragcurve = vehicle.dc;
    
    %SIMULATION SETUP
    m = m - engT*dm;    %rocket is burning fuel while bolted to the launchpad for engT seconds before it's released
    maxT = maxT - engT; %this results in loss of propellant mass and hence reduction of maximum burn time
    N = floor(maxT/dt)+1;%simulation steps
    t = zeros(N,1);     %simulation time
    %vehicle position in cartesian XYZ frame
    r = zeros(N,3);     %from Earth's center [m]
    %vehicle velocity
    v = zeros(N,3);     %relative to Earth's center [m/s]
    vv = zeros(N,1);    %magnitude vector [m/s]
    %flight angles
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
    r(1,:) = [R 0 0];
    v(1,:) = [0 10*463.3122 1000];
    vv(1) = norm(v(1,:));
    
    %MAIN LOOP
    for i=2:N
        %PHYSICS PART
        %vectors
            %local coordinates
        ru = r(i-1,:)/norm(r(i-1,:));   %radial versor -> our local "Up"
        nu = cross(r(i-1,:),v(i-1,:));
        nu = nu/norm(nu);               %normal versor -> our local "Left"
        cu = cross(nu,ru);              %circuferential versor -> our local "Forward" (prograde)
            %"navball"
        un = [r(i-1,1) r(i-1,2) 0];
        un = un/norm(un);               %local "Equatorial Up"
        nn = cross([r(i-1,1) r(i-1,2) 0],[v(i-1,1) v(i-1,2) 0]);
        nn = nn/norm(nn);               %local "North"
        en = cross(nn,un);              %local "East"
        %gravity
        Ga = mu*r(i-1,:)/norm(r(i-1,:))^3;
        %velocity
        v(i,:) = v(i-1,:) - Ga*dt;
        vv(i) = norm(v(i,:));
        %position
        r(i,:) = r(i-1,:) + v(i,:)*dt;
        %MASS&TIME
        m = m - dm*dt;
        t(i) = t(i-1) + dt;
    end;
    
    %OUTPUT
    plots = struct('t', t(1:i),...
                   'v', v,...
                   'vv', vv,...
                   'r', r);
    results = struct('Altitude', norm(r(i))-R,...
                     'Velocity', norm(v(i)),...
                     'Plots', plots);
    figure(1); clf;
    plot(vv);
    figure(2); clf;
    hold on;
    plot3(r(:,1),r(:,2),r(:,3));
    [sx,sy,sz]=sphere(20);
    plot3(R*sx,R*sy,R*sz,'g');
    scatter3(0,0,0,'g');
    %local circumferential versors
    scatter3(r(i,1),r(i,2),r(i,3),'r');
    t=zeros(2,3);
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*ru; %radial (away)
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*nu; %normal (plane change)
    plot3(t(:,1),t(:,2),t(:,3),'k');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*cu; %circumferential (prograde)
    plot3(t(:,1),t(:,2),t(:,3),'k');
    %navball versors
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*un; %away in equatorial plane
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*nn; %north
    plot3(t(:,1),t(:,2),t(:,3),'r');
    t(1,:)=r(i,:); t(2,:)=t(1,:)+1000000*en; %east
    plot3(t(:,1),t(:,2),t(:,3),'r');
    hold off;
end