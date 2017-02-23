function [error] = planeError(results, target)
%angle = PLANEERROR(results, target)
%Computes angle between target orbital plane and actually achieved plane.
%
%INPUT
%    results    Results struct as output by flightManager (NOT flightSim3D).
%    target     Target struct as output by launchTargeting.
%
%OUTPUT
%    angle      Angle between the two orbital planes.
%
%See also FLIGHTMANAGER, LAUNCHTARGETING.
    inc = results.powered(results.n).Orbit.INC;
    lan = results.powered(results.n).Orbit.LAN;
    
    Rx=[1,0,0;0,cosd(inc),-sind(inc);0,sind(inc),cosd(inc)];
    Rz=[cosd(lan),-sind(lan),0;sind(lan),cosd(lan),0;0,0,1];
    reached = (Rz*Rx*[0,0,-1]')';
    error = acosd(dot(target.normal, reached));