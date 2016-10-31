figure(9); clf;
stage = STS.powered(3);
N = stage.Plots.DEBUG.THIS;
K = 5;
t = stage.Plots.DEBUG.time(1:N);

a = stage.Plots.DEBUG.L(1:N,:);
b = stage.Plots.DEBUG.J(1:N,:);
c = stage.Plots.DEBUG.S(1:N,:);

show = 1;   %how many items to show? 1 or 2
dims = 1;   %1 shows linear plot (of norm, if passed a 4D item), 3 shows 3D plot
step = 0;   %show dots for plot points (yes or not)

s = size(a);
if dims==3 && s(2)<3
    dims = 1;
end;
if dims==3
    axis vis3d;
    hold on;
    plot3(a(:,1), a(:,2), a(:,3), 'b');
    if step
        scatter3(a(2:K-1,1), a(2:K-1,2), a(2:K-1,3), 'b');
        scatter3(a((K+1):(N-5),1), a((K+1):(N-5),2), a((K+1):(N-5),3), 'b');
    end;
    scatter3(a(1,1), a(1,2), a(1,3), 'm');
    scatter3(a(K,1), a(K,2), a(K,3), 'k');
    if show>1
        plot3(b(:,1), b(:,2), b(:,3), 'g');
        if step
            scatter3(b(2:K-1,1), b(2:K-1,2), b(2:K-1,3), 'g');
            scatter3(b((K+1):(N-5),1), b((K+1):(N-5),2), b((K+1):(N-5),3), 'g');
        end;
        scatter3(b(1,1), b(1,2), b(1,3), 'm');
        scatter3(b(K,1), b(K,2), b(K,3), 'k');
    end;
    if show>2
        plot3(c(:,1), c(:,2), c(:,3), 'r');
        if step
            scatter3(c(2:K-1,1), c(2:K-1,2), c(2:K-1,3), 'r');
            scatter3(c((K+1):(N-5),1), c((K+1):(N-5),2), c((K+1):(N-5),3), 'r');
        end;
        scatter3(c(1,1), c(1,2), c(1,3), 'm');
        scatter3(c(K,1), c(K,2), c(K,3), 'k');
    end;
    hold off;
else
    hold on;
    s = size(a);
    if s(2)>3
        adim=4;
    else
        adim=1;
    end;
    s = size(b);
    if s(2)>3
        bdim=4;
    else
        bdim=1;
    end;
    s = size(c);
    if s(2)>3
        cdim=4;
    else
        cdim=1;
    end;
    plot(t, a(:,adim), 'b');
    if step
        scatter(t(1), a(1,adim), 'm');
        scatter(t(2:K-1), a(2:K-1,adim), 'b');
        scatter(t((K+1):(N-5)), a((K+1):(N-5),adim), 'b');
    end;
    scatter(t(K), a(K,adim), 'k');
    if show>1
        plot(t, b(:,bdim), 'g');
        if step
            scatter(t(1), b(1,bdim), 'm');
            scatter(t(2:K-1), b(2:K-1,bdim), 'g');
            scatter(t((K+1):(N-5)), b((K+1):(N-5),bdim), 'g');
        end;
        scatter(t(K), b(K,bdim), 'k');
    end;
    if show>2
        plot(t, c(:,cdim), 'r');
        if step
            scatter(t(1), c(1,cdim), 'm');
            scatter(t(2:K-1), c(2:K-1,cdim), 'r');
            scatter(t((K+1):(N-5)), c((K+1):(N-5),cdim), 'r');
        end;
        scatter(t(K), c(K,cdim), 'k');
    end;
    hold off;
end;

clearvars N K t a b c show dims s adim bdim cdim stage step