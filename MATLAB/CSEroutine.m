function [r, v, last] = CSEroutine(r0, v0, dt, last)
    %Conic State Extrapolation Routine
    %Calculates vehicle state (position & velocity) after dt seconds from
    %current state (given by r0 and v0) in central gravity field with no
    %other forces present.
    %This function has been simulation-proven to give correct results!
    %Structure 'last' is used to store previous integration results to
    %speed up convergence and improve stability in next iterations:
    %   dtcp        delta tc prim; previous converged value of transfer
    %               time interval
    %   xcp         xc prim, previous converged value of x corresponding to tc
    %   A, D, E     Un function values
    if last.dtcp==0
        dtcp = dt;
    else
        dtcp = last.dtcp;
    end
    xcp = last.xcp;
    x = xcp;        %independent variable for Kepler iteration scheme
    A = last.A;
    D = last.D;
    E = last.E;
    %Program constants
    kmax = 15;      %U1 series iterator maximum value
    imax = 10;      %Kepler iterator maximum values
    global mu;
    
    %5.1 ROUTINE
    %PLATE 5-2 (p24)
    if dt>=0
        f0 = 1;
    else
        f0 = -1;
    end
    
    n = 0;
    r0m = norm(r0);
    
    f1 = f0*sqrt(r0m/mu);
    f2 = 1/f1;
    f3 = f2/r0m;
    f4 = f1*r0m;
    f5 = f0/sqrt(r0m);
    f6 = f0*sqrt(r0m);
    
    ir0 = r0/r0m;
    v0s = f1*v0;   %v0 vector with the silly dash on top
    sigma0s = dot(ir0,v0s); %sigma0 with the silly dash
    b0 = dot(v0s,v0s)-1;
    alphas = 1-b0;  %alpha with the silly dash
    
    %PLATE 5-3 (p25)
    xguess = f5*x;
    xlast = f5*xcp;
    xmin = 0;
    dts = f3*dt;    %delta t with the silly dash
    dtlast = f3*dtcp;   %delta tlast with the silly dash
    dtmin = 0;      %delta tmin with the silly dash
    
    %assuming sqrt(alphas) is never smaller than epsilon alpha
    %means orbit is not parabolic (why would it be?)
    
    xmax = 2*pi/sqrt(abs(alphas));
    
    if alphas>0
        dtmax = xmax/alphas;
        xP = xmax;      %xP with the silly dash
        Ps = dtmax;     %P with the silly dash
        while dts>=Ps
            n = n+1;
            dts = dts-Ps;
            dtlast = dtlast-Ps;
            xguess = xguess-xP;
            xlast = xlast-xP;
        end
    else
        dtmax = KTTI(xmax, sigma0s, alphas, kmax);
        if dtmax<dts
            while dtmax>=dts
                dtmin = dtmax;
                xmin = xmax;
                xmax = 2*xmax;
                dtmax = KTTI(xmax, sigma0s, alphas, kmax);
            end
        end
    end
    
    %PLATE 5-4 (p26)
    if xmin>=xguess || xguess>=xmax
        xguess = 0.5*(xmin+xmax);
    end
    
    [dtguess, ~, ~, ~] = KTTI(xguess, sigma0s, alphas, kmax);
   
    if dts<dtguess
        if xguess<xlast && xlast<xmax && dtguess<dtlast && dtlast<dtmax
            xmin = xlast;
            dtmax = dtlast;
        end
    else
        if xmin<xlast && xlast<xguess && dtmin<dtlast && dtlast<dtguess
            xmin = xlast;
            dtmin = dtlast;
        end
    end
    
    [xguess, dtguess, A, D, E] = KIL(imax, dts, xguess, dtguess, xmin, dtmin, xmax, dtmax, sigma0s, alphas, kmax, A, D, E);
    
    %PLATE 5-5 (p27)
    rs = 1 + 2*(b0*A + sigma0s*D*E);    %r with the silly dash
    b4 = 1/rs;
    
    %The following uses variables (xP, Ps) which are not even created in
    %some execution path (if aplhas>0). However, in the same path variable
    %n is never incremented above 0, so we can restate:
    if n>0
        xc = f6*(xguess+n*xP);
        dtc = f4*(dtguess+n*Ps);
    else
        xc = f6*xguess;
        dtc = f4*dtguess;
    end
    %Store converged values for the next run
    last.dtcp = dtc;
    last.xcp = xc;
    last.A = A;
    last.D = D;
    last.E = E;
    
    %Extrapolated State Vector (ROUTINE 5.3.6, PLATE 5-16 (p38)
    %Implemented inline for simplicity
    F = 1 - 2*A;
    Gs = 2*(D*E + sigma0s*A);   %G with the silly dash
    Fts = -2*b4*D*E;            %Ft with the silly dash
    Gt = 1 - 2*b4*A;
    
    r = r0m*(F*ir0 + Gs*v0s);
    v = f2*(Fts*ir0 + Gt*v0s);
end

function [t, A, D, E] = KTTI(xarg, s0s, a, kmax)
    %5.3.1 ROUTINE - Kepler Transfer Time Interval
    %PLATE 5-9 (p31)
    u1 = USS(xarg, a, kmax);
    
    zs = 2*u1;  %z with the silly dash
    E = 1 - 0.5*a*zs^2;
    w = sqrt(0.5+E/2);
    w = real(w);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    D = w*zs;
    A = D^2;
    B = 2*(E+s0s*D);
    
    Q = QCF(w);
    
    t = D*(B+A*Q);
end

function [u1] = USS(xarg, a, kmax)
    %5.3.2 ROUTINE - U1 Series Summation
    %PLATE 5-10 (p32)
    du1 = xarg/4;
    u1 = du1;
    f7 = -a*du1^2;
    k=3;
    while k<kmax
        du1 = f7*du1 / (k*(k-1));
        u1old = u1;
        u1 = u1+du1;
        if u1 == u1old
            break;
        end
        k=k+2;
    end
end

function [Q] = QCF(w)
    %5.3.3 ROUTINE - Q Continued Fraction
    %PLATE 5-11 (p33)
    if w<1
        xq=21.04-13.04*w;
    elseif w<4.625
        xq = (5/3)*(2*w+5);
    elseif w<13.846
        xq = (10/7)*(w+12);
    elseif w<44
        xq = 0.5*(w+60);
    elseif w<100
        xq = 0.25*(w+164);
    else
        xq = 70;
    end
    
    %PLATE 5-12 (p34)
    b=0;
    y=(w-1)/(w+1);
    j=floor(xq);
    b=y/(1+(j-1)/(j+2)*(1-b));
    while j>2
        j=j-1;
        b=y/(1+(j-1)/(j+2)*(1-b));
    end
    
    Q = 1/w^2 * (1 + (2-b/2) / (3*w*(w+1)));    
end

function [xguess, dtguess, A, D, E] = KIL(imax, dts, xguess, dtguess, xmin, dtmin, xmax, dtmax, s0s, a, kmax, A, D, E)
    %5.3.4 ROUTINE - Kepler Iteration Loop
    %input skips convergence criteria and "max representable scalar"
    %input adds previous values of A, D & E
    %PLATE 5-13 (p35)
    i = 1;
    while i<imax
        dterror = dts-dtguess;
        
        if abs(dterror)<0.1   %some arbitrary number, TODO try and look for hints in the paper
            break;
        end
        
        [dxs, xmin, dtmin, xmax, dtmax] = SI(dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax);
        xold = xguess;
        xguess = xguess + dxs;
        
        if xguess==xold
            break;
        end
        
    %PLATE 5-14 (p36)
        dtold = dtguess;
        
        [dtguess, A, D, E] = KTTI(xguess, s0s, a, kmax);
        
        if dtguess==dtold
            break;
        end
        
        i = i + 1;  %this line actually happens on the previous plate
    end
end

function [dxs, xmin, dtmin, xmax, dtmax] = SI(dterror, xguess, dtguess, xmin, dtmin, xmax, dtmax)
    %5.3.5 ROUTINE - Secant Iterator
    %skips passing one convergence criterion, instead define it here
    etp = 0.1;
    %PLATE 5-15 (p37)
    dtminp = dtguess-dtmin; %delta tmin prim
    dtmaxp = dtguess-dtmax;
    if abs(dtminp)<etp || abs(dtmaxp)<etp
        dxs = 0;
    else
        if dterror<0
            dxs = (xguess-xmax)*(dterror/dtmaxp);
            if (xguess+dxs)<=xmin
                dxs = (xguess-xmin)*(dterror/dtminp);
            end
            xmax = xguess;
            dtmax = dtguess;
        else
            dxs = (xguess-xmin)*(dterror/dtminp);
            if (xguess+dxs)>=xmax
                dxs = (xguess-xmax)*(dterror/dtmaxp);
            end
            xmin = xguess;
            dtmin = dtguess;
        end
    end
end