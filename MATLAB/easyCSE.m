function [r, v, last] = easyCSE(r0, v0, t, last)
%[r, v, cser] = EASYCSE(r0, v0, time, cser)
%Naive gravity integration, legacy code from the times where CSE had
%issues. This works exactly the same (except 'cser' is unused).
%See also CSEROUTINE.

    global mu;
    dt = 0.5;
    N = ceil(t/dt);
    %keep some precision minimum
    if N<150
        N = 150;
        dt = t/N;
    end
    
    v=v0;
    r=r0;
    for i=1:N
        v = v - dt*mu*r/norm(r)^3;
        r = r + dt*v;
    end