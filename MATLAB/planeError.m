function [error] = planeError(results, target)
    inc = results.powered(results.n).Orbit.INC;
    lan = results.powered(results.n).Orbit.LAN;
    
    Rx=[1,0,0;0,cosd(inc),-sind(inc);0,sind(inc),cosd(inc)];
    Rz=[cosd(lan),-sind(lan),0;sind(lan),cosd(lan),0;0,0,1];
    reached = (Rz*Rx*[0,0,-1]')';
    error = acosd(dot(target.normal, reached));