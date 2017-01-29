%launchTargeting
%For a launch from given site into specified inclination and apsis
%(altitude - MECO altitude and one apsis, opposingApsis - self explanatory)
%calculates launch azimuth, LAN of orbit passing directly over the site at
%the time of launch and builds targeting structure for UPFG with a desired
%LAN slip.
function [lan, azimuth, target] = launchTargeting(launchSite, altitude, opposingApsis, inclination, slip)
    global mu; global R; global period;
    Binertial = asind( cosd(inclination)/cosd(launchSite.lat) );%launch azimuth with no regard for Earth rotation
    vorbit = sqrt(mu/(R+altitude*1000));                        %orbital velocity magnitude
    vEarthrot = (2*pi*R/period)*cosd(launchSite.lat);           %v gained from Earth rotation
    vrotx = vorbit*sind(Binertial)-vEarthrot;
    vroty = vorbit*cosd(Binertial);
    azimuth = atan2d(vroty, vrotx);                             %corrected launch azimuth

    lan = launchSite.lon - asind(min(1,tand(90-inclination)*tand(launchSite.lat)));
    lan = mod(lan + 360 + slip, 360);                           %add slip and reduce result into 0-360 range
    
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
    target_velocity = 2 / (R+altitude*1000);
    target_velocity = target_velocity - 1 / (R+(altitude+opposingApsis)*1000/2);
    target_velocity = sqrt( mu*target_velocity );               %target velocity in steps from vis-viva equation
    target = struct('radius', R+altitude*1000, 'normal', target_plane_normal,...
                    'angle', 0, 'velocity', target_velocity);