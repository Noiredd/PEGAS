function [lan, azimuth, target] = launchTargeting(launchSite, altitude, opposingApsis, inclination, slip)
%[lan, azimuth, target] = LAUNCHTARGETING(site, altitude, apsis,
%                                                        inclination, slip)
%Creates a UPFG-compatible target structure as well as a launch azimuth for
%first stage guidance from a given launch site and definition of a target
%orbit.
%Assumes the insertion will occur in one apsis of the target orbit.
%Handles LAN specially - since the Earth does not rotate in the simulation,
%arbitrarily choosing the target LAN makes no sense. Instead, an orbit
%passing directly over the launch site at liftoff is used - its LAN is
%calculated, so for a real launch a simple difference between this and
%actually desired LAN could be calculated, and proper liftoff time adjusted.
%
%REQUIRES
%    mu         Global variable, standard gravity parameter of the body;
%               gravity constant * mass of the body (kg).
%    R          Global variable, radius of the body (m).
%    period     Global variable, period of Earth's rotation (seconds).
%
%INPUT
%    site       Struct defining the launch site.
%    altitude   Desired cutoff altitude (km above sea level). Becomes one
%               apsis of the target orbit.
%    apsis      Opposing apsis of the target orbit (km above sea level).
%    inclinationDesired inclination of the target orbit in degrees.
%    slip       Allows rotationg the target orbit forward (ie. increasing
%               its LAN), as if the lift-off occured before actually
%               passing under the target orbit (degrees). Negative values
%               mean launch is late, after the target orbit passed over the
%               launch site. Values of 1.5-2.5 work pretty good, depending
%               on vehicle. Too little or too much can break UPFG.
%
%OUTPUT
%    lan        LAN of the target orbit, including slip (degrees).
%    azimuth    Launch azimuth corrected for the velocity bonus from
%               Earth's rotation (degrees from earth to north, CCW).
%    target     Target struct, compatible with UPFG.

    global mu; global R; global period;
    
    %Calculation of the launch azimuth. Math largely based on:
    %http://www.orbiterwiki.org/wiki/Launch_Azimuth
    if inclination<launchSite.lat
        fprintf('Target inclination below launch site latitude. Assuming you know what youre doing - returning 0 azimuth.\n');
        azimuth = 0;
    else
        Binertial = asind( cosd(inclination)/cosd(launchSite.lat) );%launch azimuth with no regard for Earth rotation
        vorbit = sqrt(mu/(R+altitude*1000));                        %orbital velocity magnitude
        vEarthrot = (2*pi*R/period)*cosd(launchSite.lat);           %v gained from Earth rotation
        vrotx = vorbit*sind(Binertial)-vEarthrot;
        vroty = vorbit*cosd(Binertial);
        azimuth = atan2d(vroty, vrotx);                             %corrected launch azimuth
    end
    
    %LAN of the orbit passing directly over the launch site at lift-off.
    %Math derived from Napier's rules for spherical triangles.
    lan = launchSite.lon - asind(min(1,tand(90-inclination)*tand(launchSite.lat)));
    lan = mod(lan + 360 + slip, 360);                           %add slip and reduce result into 0-360 range
    
    %Target orbit normal vector obtained by rotation of an unit vector
    %using matrices of rotations (Ry is not used). Vector points south
    %instead of north (ie. [0,0,-1]) because UPFG requires it to point
    %opposite to a direction of vector of angular momentum for the orbit.
    Rx=[1,0,0;
        0,cosd(inclination),-sind(inclination);
        0,sind(inclination),cosd(inclination)];                 %about x for inclination (preserve zero node)
    Ry=[cosd(0),0,sind(0);
        0,1,0;
        -sind(0),0,cosd(0)];                                    %in case we needed it for something
    Rz=[cosd(lan),-sind(lan),0;
        sind(lan),cosd(lan),0;
        0,0,1];                                                 %about z for node
    target_plane_normal = (Rz*Rx*[0,0,-1]')';
    %Target velocity from the vis-viva equation, step by step for clarity.
    target_velocity = 2 / (R+altitude*1000);
    target_velocity = target_velocity - 1 / (R+(altitude+opposingApsis)*1000/2);
    target_velocity = sqrt( mu*target_velocity );
    %Flight path angle is always zero (ie. cutoff is desired to occur in
    %an apsis).
    target = struct('radius', R+altitude*1000,...
                    'normal', target_plane_normal,...
                    'angle', 0,...
                    'velocity', target_velocity);