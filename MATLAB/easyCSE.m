function [r, v, last] = easyCSE(r0, v0, t, last)
    %naive integration
    global mu;
    dt = 1;
    N = ceil(t/dt);
    v=v0;
    r=r0;
    for i=1:N
        v = v - dt*mu*r/norm(r)^3;
        r = r + dt*v;
    end